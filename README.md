## How To RUN
```bash
git clone --recurse-submodules git@github.com:realhonbo/rtp-kernel.git
cd rtp-kernel/samples/libopencm3-template
make -C libopencm3 -j$(nproc)  # ! Only need make once
make -C src
```
the bin and elf file will create in src/


## Feature
1. RR scheduler with O(1) lookup
2. Ticket based spinlock
