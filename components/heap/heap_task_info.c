// Copyright 2018 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <multi_heap.h>
#include <string.h>
#include "multi_heap_internal.h"
#include "heap_private.h"
#include "esp_heap_task_info.h"

#ifdef CONFIG_HEAP_TASK_TRACKING
static SLIST_HEAD(heap_task_stat_ll, heap_task_stat) task_stats;

#ifdef CONFIG_HEAP_TASK_TRACKING_INCLUDE_TASKNAME
/* Function is useful when active allocations' task is deleted */
static const char *heap_caps_get_task_name(TaskHandle_t task)
{
    heap_task_stat_t *task_info = NULL;
    SLIST_FOREACH(task_info, &task_stats, next) {
        if (task_info->task == task) {
            return task_info->taskname;
        }
    }
    return "NotFound";
}
#endif

IRAM_ATTR void heap_caps_update_per_task_info_alloc(multi_heap_handle_t heap, size_t size, int caps)
{
    TaskHandle_t task = xTaskGetCurrentTaskHandle();

    int type = ((caps & MALLOC_CAP_8BIT) == MALLOC_CAP_8BIT) ? 0 : 1;

    heap_task_stat_t *task_info = NULL;
    SLIST_FOREACH(task_info, &task_stats, next) {
        if (task_info->task == task) {
            task_info->current[type] += size;
            if (task_info->current[type] > task_info->peak[type]) {
                task_info->peak[type] = task_info->current[type];
            }
            if (!task_info->min_block) {
                task_info->min_block = size;
            }
            if (task_info->min_block > size) {
                task_info->min_block = size;
            }
            if (task_info->max_block < size) {
                task_info->max_block = size;
            }
            return;
        }
    }

    task_info = multi_heap_malloc(heap, sizeof(heap_task_stat_t));
    if (!task_info) {
        return;
    }
    memset(task_info, 0, sizeof(heap_task_stat_t));
    task_info->task = task;
    task_info->current[type] = size;
    task_info->peak[type] = size;
    task_info->min_block = size;
    task_info->max_block = size;
#ifdef CONFIG_HEAP_TASK_TRACKING_INCLUDE_TASKNAME
    if (!task) {
        strlcpy(task_info->taskname, "Pre-schedular", CONFIG_HEAP_MAX_TASK_NAME_LEN);
    } else {
        strlcpy(task_info->taskname, pcTaskGetTaskName(task), CONFIG_HEAP_MAX_TASK_NAME_LEN);
    }
#endif
    SLIST_INSERT_HEAD(&task_stats, task_info, next);
}

IRAM_ATTR void heap_caps_update_per_task_info_free(multi_heap_handle_t heap, void *ptr, int caps)
{
    TaskHandle_t task = (TaskHandle_t)multi_heap_get_block_owner_from_ptr(heap, ptr);
    if (!task) {
        return;
    }
    int type = ((caps & MALLOC_CAP_8BIT) == MALLOC_CAP_8BIT) ? 0 : 1;
    heap_task_stat_t *task_info = NULL;
    SLIST_FOREACH(task_info, &task_stats, next) {
        if (task_info->task == task) {
            task_info->current[type] -= multi_heap_get_allocated_size(heap, ptr);
        }
    }
}

IRAM_ATTR void heap_caps_update_per_task_info_realloc(multi_heap_handle_t heap, size_t old_size, TaskHandle_t old_task, size_t new_size, int caps)
{
    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    bool task_in_list = false;

    int type = ((caps & MALLOC_CAP_8BIT) == MALLOC_CAP_8BIT) ? 0 : 1;

    heap_task_stat_t *task_info = NULL;
    SLIST_FOREACH(task_info, &task_stats, next) {
        if (task_info->task == task) {
            task_info->current[type] += new_size;
            if (task_info->current[type] > task_info->peak[type]) {
                task_info->peak[type] = task_info->current[type];
            }
            if (!task_info->min_block) {
                task_info->min_block = new_size;
            }
            if (task_info->min_block > new_size) {
                task_info->min_block = new_size;
            }
            if (task_info->max_block < new_size) {
                task_info->max_block = new_size;
            }
            task_in_list = true;
        }
        if (task_info->task == old_task) {
            task_info->current[type] -= old_size;
        }
    }

    if (task_in_list)
        return;

    task_info = multi_heap_malloc(heap, sizeof(heap_task_stat_t));
    if (!task_info) {
        return;
    }
    memset(task_info, 0, sizeof(heap_task_stat_t));
    task_info->task = task;
    task_info->current[type] =  new_size;
    task_info->peak[type] = new_size;
    task_info->min_block = new_size;
    task_info->max_block = new_size;
#ifdef CONFIG_HEAP_TASK_TRACKING_INCLUDE_TASKNAME
    if (!task) {
        strlcpy(task_info->taskname, "Pre-schedular", CONFIG_HEAP_MAX_TASK_NAME_LEN);
    } else {
        strlcpy(task_info->taskname, pcTaskGetTaskName(task), CONFIG_HEAP_MAX_TASK_NAME_LEN);
    }
#endif
    SLIST_INSERT_HEAD(&task_stats, task_info, next);
}

/*
 * Return per-task heap allocation totals and lists of blocks.
 *
 * For each task that has allocated memory from the heap, return totals for
 * allocations within regions matching one or more sets of capabilities.
 *
 * Optionally also return an array of structs providing details about each
 * block allocated by one or more requested tasks, or by all tasks.
 *
 * Returns the number of block detail structs returned.
 */
size_t heap_caps_get_per_task_info(heap_task_info_params_t *params)
{
    heap_t *reg;
    heap_task_block_t *blocks = params->blocks;
    size_t count = *params->num_totals;
    size_t remaining = params->max_blocks;

    // Clear out totals for any prepopulated tasks.
    if (params->totals) {
        for (size_t i = 0; i < count; ++i) {
            for (size_t type = 0; type < NUM_HEAP_TASK_CAPS; ++type) {
                params->totals[i].size[type] = 0;
                params->totals[i].count[type] = 0;
            }
        }
    }

    SLIST_FOREACH(reg, &registered_heaps, next) {
        multi_heap_handle_t heap = reg->heap;
        if (heap == NULL) {
            continue;
        }

        // Find if the capabilities of this heap region match on of the desired
        // sets of capabilities.
        uint32_t caps = get_all_caps(reg);
        uint32_t type;
        for (type = 0; type < NUM_HEAP_TASK_CAPS; ++type) {
            if ((caps & params->mask[type]) == params->caps[type]) {
                break;
            }
        }
        if (type == NUM_HEAP_TASK_CAPS) {
            continue;
        }

        multi_heap_block_handle_t b = multi_heap_get_first_block(heap);
        multi_heap_internal_lock(heap);
        for ( ; b ; b = multi_heap_get_next_block(heap, b)) {
#ifdef CONFIG_HEAP_TASK_TRACKING_INCLUDE_FREE_BLOCKS
            bool is_allocated = true;
#endif
            if (multi_heap_is_free(b)) {
#ifdef CONFIG_HEAP_TASK_TRACKING_INCLUDE_FREE_BLOCKS
                is_allocated = false;
#else
                continue;
#endif
            }
            void *p = multi_heap_get_block_address(b);  // Safe, only arithmetic
            size_t bsize;
#ifdef CONFIG_HEAP_TASK_TRACKING_INCLUDE_FREE_BLOCKS
            if (!is_allocated) {
                bsize = multi_heap_get_free_size(b);
            } else
#endif
            bsize = multi_heap_get_allocated_size(heap, p); // Validates
            TaskHandle_t btask = (TaskHandle_t)multi_heap_get_block_owner(b);

#ifdef CONFIG_HEAP_TASK_TRACKING_INCLUDE_FREE_BLOCKS
            if (is_allocated) {
#endif
            // Accumulate per-task allocation totals.
            if (params->totals) {
                size_t i;
                for (i = 0; i < count; ++i) {
                    if (params->totals[i].task == btask) {
                        break;
                    }
                }
                if (i < count) {
                    params->totals[i].size[type] += bsize;
                    params->totals[i].count[type] += 1;
                }
                else {
                    if (count < params->max_totals) {
                        params->totals[count].task = btask;
                        params->totals[count].size[type] = bsize;
                        params->totals[i].count[type] = 1;
                        ++count;
                    }
                }
            }
#ifdef CONFIG_HEAP_TASK_TRACKING_INCLUDE_FREE_BLOCKS
            }
#endif

            // Return details about allocated blocks for selected tasks.
            if (blocks && remaining > 0) {
                if (params->tasks) {
                    size_t i;
                    for (i = 0; i < params->num_tasks; ++i) {
                        if (btask == params->tasks[i]) {
                            break;
                        }
                    }
                    if (i == params->num_tasks) {
                        continue;
                    }
                }
                blocks->task = btask;
                blocks->address = p;
                blocks->size = bsize;
                ++blocks;
                --remaining;
#ifdef CONFIG_HEAP_TASK_TRACKING_INCLUDE_FREE_BLOCKS
                blocks->is_allocated = is_allocated;
#endif
            }
        }
        multi_heap_internal_unlock(heap);
    }
    *params->num_totals = count;
    return params->max_blocks - remaining;
}

size_t heap_caps_get_next_task_stat(heap_task_stat_t *task, const bool start)
{
    static heap_task_stat_t *task_info;

    if (!task_info && !start)
        return 0;

    if (start) {
        task_info = SLIST_FIRST(&task_stats);
        if (!task_info)
            return 0;
    } else {
        task_info = SLIST_NEXT(task_info, next);
        /* Exhausted the list */
        if (!task_info) {
            return 0;
        }
    }
    
    memcpy(task, task_info, sizeof(heap_task_stat_t));
    return sizeof(heap_task_stat_t);
}

size_t heap_caps_get_next_block_info(heap_task_block_t *block, const bool start)
{
    static heap_t *reg;
    static multi_heap_handle_t heap;
    static multi_heap_block_handle_t b;

    if (start) {
        reg = SLIST_FIRST(&registered_heaps);
        if (!reg)
            return 0;
        heap = reg->heap;
        if (!heap || !(b = multi_heap_get_first_block(heap)))
            return 0;
    }

    while(!b) {
        reg = SLIST_NEXT(reg, next);
        if (!reg) {
            return 0;
        }
        heap = reg->heap;
        if (!heap)
            continue;
        b = multi_heap_get_first_block(heap);
    }

    multi_heap_internal_lock(heap);
    block->address = multi_heap_get_block_address(b);  // Safe, only arithmetic
    if (multi_heap_is_free(b)) {
#ifdef CONFIG_HEAP_TASK_TRACKING_INCLUDE_FREE_BLOCKS
        block->is_allocated = false;
#endif
        block->size = multi_heap_get_free_size(b);
        block->task = (TaskHandle_t)0;
    } else {
#ifdef CONFIG_HEAP_TASK_TRACKING_INCLUDE_FREE_BLOCKS
        block->is_allocated = true;
#endif
        block->size = multi_heap_get_allocated_size(heap, block->address);
        block->task = (TaskHandle_t)multi_heap_get_block_owner(b);
#ifdef CONFIG_HEAP_TASK_TRACKING_INCLUDE_TASKNAME
        strlcpy(block->taskname, heap_caps_get_task_name(block->task), CONFIG_HEAP_MAX_TASK_NAME_LEN);
#endif
    }

    b = multi_heap_get_next_block(heap, b);
    multi_heap_internal_unlock(heap);
    return 1;
}

#endif // CONFIG_HEAP_TASK_TRACKING
