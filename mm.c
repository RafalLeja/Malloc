/*
 * Rafał Leja 340879
 *
 * version with a single free linked list and optimized boundary tags
 *
 *
 * linked list type ->
 *          | FIFO               | LIFO                | ADDRSORT
 * FIRSTFIT | 75.6; 92.06; 7647  | 78.3; 93.53; 7701   | 76.8; 93.82; 9743
 * NEXTFIT  | 75.6; 92.06; 11727 | 78.3; 93.53; 11817  | 76.8; 93.81; 13858
 * BESTFIT  | 77.1; 95.36; 11553 | 77.1; 95.43; 11583  | 77.0; 95.28; 13606
 * ^ fit policy;                     ^     ^      ^ Instructions per operation
 *                                   |     L Total memory utilization
 *                                   L Weighted memory utilization
 *
 * ALLOCATED BLOCK:
 * ----------------
 *  header: size - 28 bits, flags - 4 bits;
 *    flags: 0 | 0 | PREVFREE | USED/FREE - 1 if block is allocate, 0 if free
 *                      L 1 if previous block is free, 0 if not
 *
 *  payload: data
 *  footer: size - 28 bits, flags - 4 bits;
 *    the same as header
 * ----------------
 *
 *
 * FREE BLOCK:
 * ----------------
 *  header: size - 28 bits, flags - 4 bits;
 *    flags: NEXTNULL | PREVNULL | PREVFREE | USED/FREE - 1 if block is
 allocated, 0 if free
 *              |          |           L 1 if previous block is free, 0 if not
 *              |          L 1 if previous link address is NULL, 0 if not
 *              L 1 if next link address is NULL, 0 if not
 *  prev link: 28 bit compressed address of the previous free block link
 *  next link: 28 bit compressed address of the next free block link
 *
 *  some leftover data
 *
 *  footer: size - 28 bits, flags - 4 bits;
 *   the same as header except the flags may not be set
 * ----------------
 *
 * ADDRESS COMPRESSION:
 *  Since the heap is smaller then 4GB = 2^32
 *    that means that if the first block's address is 0x0,
 *    than the last block's can't be larger than 2^32.
 *
 *  Thus if we use the beginning of the heap,
 *    as a anchor of our relative address,
 *    we can store it in one word.
 *
 *  So:
 *    - the relative address is: absolute_addres - heap_start
 *    - the absolute address is: relative_addres + heap_start
 *
 * allocating a block:
 * 1. find a block that is big enough using the find_fit function
 * 2. if the block is the exact size, remove it from the free list and return it
 * 3. if the block is bigger, split it into two blocks, one of the requested
 size and the other with the remaining size
 * 4. if the block is not found, and the last block is free, extend it
 * 5. if the last block is not free, extend the heap
 *
 * freeing a block:
 * 1. check if the previous block is free, if so, extend the previous block
 * 2. check if the next block is free, if so, extend the current block
 * 3. make sure the block is in the free list
 * 4. set the header, footer on this block, and PREVFREE flag on the next block
 *
 */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>

#include "mm.h"
#include "memlib.h"

/* If you want debugging output, use the following macro.  When you hand
 * in, remove the #define DEBUG line. */
// #define DEBUG
#ifdef DEBUG
#define debug(...) printf(__VA_ARGS__)
#else
#define debug(...)
#endif

/* do not change the following! */
#ifdef DRIVER
/* create aliases for driver tests */
#define malloc mm_malloc
#define free mm_free
#define realloc mm_realloc
#define calloc mm_calloc
#endif /* def DRIVER */

#define MAX(x, y) ((x) > (y) ? (x) : (y))

// ==== PARAMETERS
// == linked list type:

// #define FIFO
#define LIFO
// #define ADDRSORT

// == find fit type:
#define FIRSTFIT
// #define NEXTFIT
// #define BESTFIT

typedef struct {
  int32_t header;
  /*
   * We don't know what the size of the payload will be, so we will
   * declare it as a zero-length array.  This allow us to obtain a
   * pointer to the start of the payload.
   */
  uint8_t payload[];
} block_t;

#ifdef NEXTFIT
static block_t *last_fit;
#endif

// globals and types from mm-implicit.c
typedef int32_t word_t; /* Heap is bascially an array of 4-byte words. */

typedef enum {
  FREE = 0,     /* Block is free */
  USED = 1,     /* Block is used */
  PREVFREE = 2, /* Previous block is free (optimized boundary tags) */
  PREVNULL = 4, /* Previous free link is NULL */
  NEXTNULL = 8  /* Next free link is NULL */
} bt_flags;

static block_t *heap_start; /* Address of the first block */
static block_t *heap_end;   /* Address past last byte of last block */
static block_t *last;       /* Points at last block */

static block_t *freelist_start; // absolute address of the first free block
static block_t *freelist_end;   // absolute address of the last free block

// ======= general functions =======

// round up to the nearest multiple of ALIGNMENT
static inline size_t round_up(size_t size) {
  return (size + ALIGNMENT - 1) & -ALIGNMENT;
}

// ======= block parameter functions =======

// get the size of the block from the header
static inline size_t get_size(block_t *block) {
  return block->header & -16;
}

// check if the block is free
static inline bool is_free(block_t *block) {
  return (block->header & 1) == FREE;
}

// check if the previous block is free
static inline bool is_prev_free(block_t *block) {
  return (block->header & 2) == PREVFREE;
}

// ======= block movement functions =======

// get the next block
static inline block_t *next_block(block_t *curr) {
  return (void *)curr + get_size(curr);
}

// get the previous block
static inline block_t *prev(block_t *curr) {
  word_t *footer = (word_t *)((long)curr - sizeof(word_t));
  size_t size = *footer & -4;
  return (block_t *)((long)curr - size);
}

// ======= block setting functions =======

// set the header of the block, without changing the flags related to freelist
static inline void set_header(block_t *block, size_t size,
                              bt_flags is_allocated) {

  bt_flags free_flags = block->header & (PREVNULL | NEXTNULL);
  block->header = (int32_t)(size | is_allocated | free_flags);
}

// set the footer of the block
static inline void set_footer(block_t *block, size_t size,
                              bt_flags is_allocated) {

  word_t *footer = (void *)block + get_size(block) - sizeof(word_t);
  *footer = size | is_allocated;
}

// set the next block's PREVFREE flag, or remove it
static inline void set_next(block_t *c_block, bt_flags flags) {

  block_t *n_block = next_block(c_block);

  if (n_block != (block_t *)heap_end) {
    if (flags == PREVFREE) {
      n_block->header |= PREVFREE;
    } else {
      n_block->header &= -3;
    }
  }
}

// ======= freelist helper functions =======

// get the compressed address of the block
static inline word_t rel_addr(block_t *block) {
  return (word_t)((long)block - (long)heap_start);
}

// get the absolute address of the block
static inline block_t *abs_addr(int rel_addr) {
  return (block_t *)(((long)(rel_addr & 0xFFFFFFFF)) + (long)heap_start);
}

// ======= freelist setting functions =======

// set the previous link address of the block, or if the address is NULL, set
// the appropriate flag
static inline void set_prev_link(block_t *block, block_t *prev) {
  if (prev != NULL) {

    *(((word_t *)block) + 1) = (int)rel_addr(prev);
    // remove the PREVNULL flag
    block->header &= -5;
  } else {
    block->header |= PREVNULL;
  }
}

// set the next link address of the block, or if the address is NULL, set the
// appropriate flag
static inline void set_next_link(block_t *block, block_t *next) {
  if (next != NULL) {

    *(((word_t *)block) + 2) = (int)rel_addr(next);

    block->header &= -9;
  } else {
    block->header |= NEXTNULL;
  }
}

// ======= freelist movement functions =======

// get the previous link address of the block, or NULL if the flag is set
static inline void *prev_link(block_t *block) {
  if (block->header & PREVNULL) {
    return NULL;
  }
  return abs_addr(*((word_t *)block + 1));
}

// get the next link address of the block, or NULL if the flag is set
static inline void *next_link(block_t *block) {
  if (block->header & NEXTNULL) {
    return NULL;
  }
  return abs_addr(*((word_t *)block + 2));
}

// ======= freelist functions =======

// change the addres of the block in the freelist
static inline void move_link(block_t *old_block, block_t *new_block) {

  block_t *prev = prev_link(old_block);
  block_t *next = next_link(old_block);

#ifdef NEXTFIT
  if (old_block == last_fit) {
    last_fit = new_block;
  }
#endif

  if (prev != NULL) {
    set_next_link(prev, new_block);
    set_prev_link(new_block, prev);
  } else {
    freelist_start = new_block;
    set_prev_link(new_block, NULL);
  }

  if (next != NULL) {
    set_prev_link(next, new_block);
    set_next_link(new_block, next);
  } else {
    freelist_end = new_block;
    set_next_link(new_block, NULL);
  }
}

// remove the block from the freelist
static inline void remove_link(block_t *block) {

  block_t *prev = prev_link(block);
  block_t *next = next_link(block);

#ifdef NEXTFIT
  if (block == last_fit) {
    last_fit = NULL;
  }
#endif

  if (prev != NULL) {
    set_next_link(prev, next);
  } else {
    freelist_start = next;
  }

  if (next != NULL) {
    set_prev_link(next, prev);
  } else {
    freelist_end = prev;
  }
}

// insert the block to the freelist
static void insert_free(block_t *block) {
  // if the freelist is empty, set the start and end to the block
  if (freelist_start == NULL) {
    freelist_start = block;
    freelist_end = block;
    set_prev_link(block, NULL);
    set_next_link(block, NULL);
    return;
  }

#ifdef FIFO
  block_t *curr = freelist_start;
  freelist_start = block;
  set_next_link(block, curr);
  set_prev_link(block, NULL);
  set_prev_link(curr, block);
  return;
#endif

#ifdef LIFO
  block_t *curr = freelist_end;
  freelist_end = block;
  set_prev_link(block, curr);
  set_next_link(block, NULL);
  set_next_link(curr, block);
  return;
#endif

#ifdef ADDRSORT
  block_t *curr = freelist_start;
  // find the right place
  while (curr < block && curr != freelist_end) {
    curr = next_link(curr);
  }

  if (curr == freelist_start) {
    freelist_start = block;
    set_next_link(block, curr);
    set_prev_link(block, NULL);
    set_prev_link(curr, block);
    return;
  } else if (curr == freelist_end) {
    freelist_end = block;
    set_prev_link(block, curr);
    set_next_link(block, NULL);
    set_next_link(curr, block);
    return;
  } else {
    block_t *prev = prev_link(curr);
    set_next_link(prev, block);
    set_prev_link(block, prev);
    set_next_link(block, curr);
    set_prev_link(curr, block);
    return;
  }
#endif
}

// find the best free block for the size
static void *find_fit(size_t size) {

  if (freelist_start == NULL) {
    return heap_end;
  }

#ifdef FIRSTFIT
  block_t *curr = freelist_start;
  while (curr != freelist_end) {
    if (get_size(curr) >= size) {
      return curr;
    }
    curr = next_link(curr);
  }
  if (get_size(curr) >= size) {
    return curr;
  }
  return heap_end;
#endif

#ifdef NEXTFIT
  if (last_fit == NULL) {
    last_fit = freelist_start;
  }
  block_t *curr = last_fit;
  block_t *start = curr;
  bool looped = false;

  while (curr != start || !looped) {
    if (get_size(curr) >= size) {
      last_fit = curr;
      return curr;
    }
    if (curr == freelist_end) {
      curr = freelist_start;
      looped = true;
    } else {
      curr = next_link(curr);
    }
  }
  last_fit = NULL;
  return heap_end;
#endif

#ifdef BESTFIT
  block_t *curr = freelist_start;
  block_t *best = heap_end;
  size_t best_size = -1;

  while (curr != freelist_end) {
    if (get_size(curr) >= size && get_size(curr) < best_size) {
      best = curr;
      best_size = get_size(curr);
    }
    curr = next_link(curr);
    if (best_size == size) {
      return best;
    }
  }
  if (get_size(curr) >= size && get_size(curr) < best_size) {
    best = curr;
  }
  return best;
#endif
}

// ======= mm.c functions =======

/*
 * mm_init - Called when a new trace starts.
 */
int mm_init(void) {
  /* Pad heap start so first payload is at ALIGNMENT. */
  debug("ALIGNMENT: %d\n", ALIGNMENT);
  if ((long)mem_sbrk(ALIGNMENT - offsetof(block_t, payload)) < 0)
    return -1;

  heap_start = heap_end = last = (mem_heap_hi() + 1);

  freelist_start = NULL;
  freelist_end = NULL;

  return 0;
}

/*
 * malloc - Allocate a block by incrementing the brk pointer.
 *      Always allocate a block whose size is a multiple of the alignment.
 */
void *malloc(size_t size) {
  // round the size to the nearest multiple of ALIGNMENT
  size = MAX(round_up(sizeof(block_t) + size), 4 * sizeof(word_t));
  debug("======!malloc! rounded size: %ld\n", size);

  // try to find a big enough free block
  block_t *curr = (block_t *)find_fit(size);

  // if 'find_fit' found a block
  if (curr < (block_t *)heap_end) {

    // block is the exact size
    if (get_size(curr) == size || get_size(curr) - size < sizeof(block_t)) {

      // removing the block, and changing the header
      remove_link(curr);
      set_header(curr, size, USED);
      set_next(curr, 0);
      return curr->payload;
    }
    // block is bigger, so we need to split it
    else {

      size_t whole_size = get_size(curr);
      remove_link(curr);

      // create the allocated block
      set_header(curr, size, USED);

      block_t *s_half = next_block(curr);
      if (curr == last) {
        last = s_half;
      }

      // create the free block
      set_header(s_half, whole_size - size, FREE);
      set_footer(s_half, whole_size - size, FREE);
      insert_free(s_half);
      set_next(s_half, PREVFREE);

      return curr->payload;
    }
  }

  // we need to extend the heap, but the last block is free, so we can extend it
  if (is_free(last) && last != heap_start) {

    curr = last;
    size_t last_size = get_size(curr);
    mem_sbrk(size - last_size);
    remove_link(last);
    set_header(curr, size, USED);
    heap_end = next_block(curr);
    return curr->payload;
  }
  // we need to extend the heap
  else {

    block_t *block = mem_sbrk(size);
    if ((long)block < 0) {
      // ran out of memory
      debug("======!malloc! out of memory\n");
      return NULL;
    }

    set_header(block, size, USED);
    last = block;
    heap_end = next_block(last);

    return block->payload;
  }

  // something went wrong, return NULL
  debug("======!malloc! something went wrong\n");
  return NULL;
}

/*
 * free - We don't know how to free a block.  So we ignore this call.
 *      Computers have big memories; surely it won't be a problem.
 */
void free(void *ptr) {
  debug("======!free! ptr: %lx\n", (long)ptr);
  if (ptr == NULL)
    return;

  block_t *block = (block_t *)((word_t *)ptr - 1);
  size_t size = get_size(block);

  bool coalesced = false;
  // check if block to the left is empty
  if (is_prev_free(block)) {
    if (prev(block) >= heap_start) {
      if (block == last) {
        last = prev(block);
      }

      size += get_size(prev(block));
      block = prev(block);
      set_header(block, size, FREE);
      set_footer(block, size, FREE);

      coalesced = true;
    }
  }

  // check if block to the right is empty
  if (next_block(block) < heap_end && is_free(next_block(block))) {

    if (next_block(block) == last) {
      last = block;
    }

    size += get_size(next_block(block));
    if (coalesced) {
      remove_link(next_block(block));
    } else {
      move_link(next_block(block), block);
      coalesced = true;
    }
    set_header(block, size, FREE);
    set_footer(block, size, FREE);
  }

  if (!coalesced) {
    set_header(block, size, FREE);
    set_footer(block, size, FREE);
    insert_free(block);
  }

  set_next(block, PREVFREE);
}

/*
 * realloc - Change the size of the block by mallocing a new block,
 *      copying its data, and freeing the old block.
 **/
void *realloc(void *old_ptr, size_t size) {
  /* If size == 0 then this is just free, and we return NULL. */
  if (size == 0) {
    free(old_ptr);
    return NULL;
  }

  /* If old_ptr is NULL, then this is just malloc. */
  if (!old_ptr)
    return malloc(size);

  void *new_ptr = malloc(size);

  /* If malloc() fails, the original block is left untouched. */
  if (!new_ptr)
    return NULL;

  /* Copy the old data. */
  block_t *block = old_ptr - offsetof(block_t, payload);
  size_t old_size = get_size(block);
  if (size < old_size)
    old_size = size;
  memcpy(new_ptr, old_ptr, old_size);

  /* Free the old block. */
  free(old_ptr);

  return new_ptr;
}

/*
 * calloc - Allocate the block and set it to zero.
 */
void *calloc(size_t nmemb, size_t size) {
  size_t bytes = nmemb * size;
  void *new_ptr = malloc(bytes);

  /* If malloc() fails, skip zeroing out the memory. */
  if (new_ptr)
    memset(new_ptr, 0, bytes);

  return new_ptr;
}

// ======= mm_checkheap functions =======

// print the blocks
static void print_blocks() {
  block_t *curr = heap_start;
  while (curr < heap_end) {
    debug("block: %lx, size: %ld, flags %x\n", (long)curr, get_size(curr),
          curr->header & 3);
    curr = next_block(curr);
  }
}

// print the freelist
static void print_freelist() {
  block_t *curr = freelist_start;
  while (curr != NULL) {
    debug("block: %lx, size: %ld, flags %x\n", (long)curr, get_size(curr),
          curr->header & 0xf);
    debug("prev: %lx, next: %lx\n", (long)prev_link(curr),
          (long)next_link(curr));
    curr = next_link(curr);
  }
}

/*
 * mm_checkheap - So simple, it doesn't need a checker!
 */
void mm_checkheap(int verbose) {
  // print the variables
  if (verbose) {
    debug("heap_start: %lx\nheap_end: %lx\nlast: %lx\n", (long)heap_start,
          (long)heap_end, (long)last);
    debug("mem_heap_lo: %lx\nmem_heap_hi: %lx\n", (long)mem_heap_lo(),
          (long)mem_heap_hi());

    debug("freelist_start: %lx\nfreelist_end: %lx\n", (long)freelist_start,
          (long)freelist_end);
  }

  // iterate through the blocks, assert the invariants
  block_t *curr = heap_start;
  block_t *prev = NULL;

  bool prev_free = false;
  int free_links = 0;
  int free_blocks = 0;
  int i = 0;

  while (curr < heap_end) {
    // check for two empty blocks in a row
    if (is_free(curr) && prev_free) {
      debug("two free blocks in a row\n");
      print_blocks();
      assert(false);
    }
    if (is_free(curr)) {
      free_blocks++;
      // check if the freelist links are correct
      if ((curr->header & PREVNULL) == 0) {

        if (prev_link(curr) < (void *)heap_start ||
            prev_link(curr) > (void *)heap_end) {
          debug("error at block %lx with header %x, prev link %lx\n",
                (long)curr, curr->header, (long)prev_link(curr));

          print_freelist();
        }
        assert(prev_link(curr) >= (void *)heap_start);
        assert(prev_link(curr) < (void *)heap_end);

      } else {
        assert(prev_link(curr) == NULL);
      }

      if ((curr->header & NEXTNULL) == 0) {

        if (next_link(curr) < (void *)heap_start ||
            next_link(curr) > (void *)heap_end) {
          debug("error at block %lx with header %x, next link %lx\n",
                (long)curr, curr->header, (long)next_link(curr));
          print_freelist();
        }
        assert(next_link(curr) >= (void *)heap_start);
        assert(next_link(curr) < (void *)heap_end);

      } else {
        assert(next_link(curr) == NULL);
      }
    }
    prev_free = is_free(curr);
    prev = curr;
    curr = next_block(curr);
    i++;
  }
  // make sure the last block is the last block
  assert(prev == last || i == 0);

  // iterate through the freelist, assert the invariants
  curr = freelist_start;
  while (curr != NULL) {
    free_links++;
    // check if the block is free
    assert((curr->header & USED) == 0);
    curr = next_link(curr);
  }

  // if the number of free links is different than the number of free blocks,
  // print the blocks
  if (free_links != free_blocks) {
    debug("free_links: %d, free_blocks: %d\n", free_links, free_blocks);
    curr = heap_start;
    while (curr < heap_end) {
      debug("block: %lx, size: %ld, flags %x\n", (long)curr, get_size(curr),
            curr->header & 3);
      curr = next_block(curr);
    }
  }
  assert(free_links == free_blocks);
}
