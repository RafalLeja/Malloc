## Malloc

Dynamic memory allocator, written in C, based on a template, done for a university project for the course of Operating Systems.

## How it works

The allocator manages the [heap](https://en.wikipedia.org/wiki/Memory_management#HEAP), a contiguous block of memory, by dividing it into blocks of different sizes, providing the user an interface to allocate and deallocate memory.

It uses two linked-lists one for free blocks and one for all blocks, to speed up the allocation and deallocation process.

It has three diffrent allocation policies:
- First Fit
- Best Fit
- Worst Fit

And three diffrent queue policies:
- FIFO
- LIFO
- Ordered by address

## How to use

use `make` to compile the program,

`./grade` to run the tests,

`./mdriver` to run the driver,

`./mdriver -h` to see the help menu.