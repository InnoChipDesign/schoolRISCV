        .text

start:  li      t1, 3           # bitmask for addr offset
        li      t2, 2044        # bitmask for addr (0111 1111 1100)
        li      t5, 255         # bitmask for output

loop:   lw      t0, 0xc  (zero) # load state of the switches
        mv      t6, t0          # copy switches state into t6
        and     t0, t0, t5      # cut off first two switches
        and     t3, t0, t2      # extract base memory address
        and     t4, t0, t1      # extract address offset
        lw      t0, 0x0  (t3)   # load from base memory address
        add     t4, t4, t4      # 2 * offset
        add     t4, t4, t4      # 4 * offset
        add     t4, t4, t4      # 8 * offset
        srl     t0, t0, t4      # shift value according to offset*8
        and     t0, t0, t5      # mask higher bits
        sw      t0, 0x4  (zero) # store result on LEDs

        li      t0, 256
        and     t0, t6, t0      # check state of `write` switch
        beq     t0, zero, loop  # if zero - loop back

        # write zeros to location
        # lw      t0, 0x0  (t3)   # load from base memory address
        # mv      t6, zero       
        # addi    t6, t6, -1      # load #ffffffff into t6
        mv      t0, zero
        addi    t0, t0, -1
        sw      t0, 0x0  (t3)

        beq     zero, zero, loop