        .text

start:  mv      a0, zero        # set counter to zero
        li      t1, 1           # bit mask for inc
        li      t2, 2           # bit mask for dec

count:  beq, zero, zero, display
ret:    lw      t0, 0x14 (zero) # load state of the keys
        lw      t5, 0xc  (zero) # load state of the switches
        and     t3, t0, t1      # is inc set?
        and     t4, t0, t2      # is dec set?
        
        beq     t3, zero, no_inc # skip increment if t3 is zero
        add     a0, a0, t5       # increment counter

no_inc: beq     t4, zero, no_dec
        sub     a0, a0, t5       # decrement counter

no_dec: beq     zero, zero, count



display:                        # display counter value (a0) on HEX display
        sw      t0, 0x400 (zero)        # save t0
        sw      t1, 0x404 (zero)        # save t1
        sw      t2, 0x408 (zero)        # save t2
        sw      t3, 0x40c (zero)        # save t3
        sw      t4, 0x410 (zero)        # save t4
        sw      t5, 0x414 (zero)        # save t5
        sw      t6, 0x418 (zero)        # save t6
        sw      a1, 0x41c (zero)        # save a1
        
        mv      t0, a0                  # make temporary vairable, init with counter value
        li      t1, 15                  # bit mask for first 4 bits
        li      t3, 0                   # hex index
        li      t5, 8                   # hex count
        li      t6, 3                   # bit mask for not aligned addresses
        li      a1, 0                   # temp value to write to memory

digit:  and     t2, t0, t1              # extract first 4 bits (current digit)
                                        # now t2 contains the value of the hex digit we need to print

        li      t4, 0
        beq     t2, t4, l0
        li      t4, 1
        beq     t2, t4, l1
        li      t4, 2
        beq     t2, t4, l2
        li      t4, 3
        beq     t2, t4, l3
        li      t4, 4
        beq     t2, t4, l4
        li      t4, 5
        beq     t2, t4, l5
        li      t4, 6
        beq     t2, t4, l6
        li      t4, 7
        beq     t2, t4, l7
        li      t4, 8
        beq     t2, t4, l8
        li      t4, 9
        beq     t2, t4, l9
        li      t4, 0xa
        beq     t2, t4, la
        li      t4, 0xb
        beq     t2, t4, lb
        li      t4, 0xc
        beq     t2, t4, lc
        li      t4, 0xd
        beq     t2, t4, ld
        li      t4, 0xe
        beq     t2, t4, le
        li      t4, 0xf
        beq     t2, t4, lf

        li      t4, 0                   # hex value for `0`
        beq     zero, zero, pro
l0:     li      t4, 192                 # hex value for `0`
        beq     zero, zero, pro
l1:     li      t4, 249                 # hex value for `1`
        beq     zero, zero, pro
l2:     li      t4, 164          
        beq     zero, zero, pro
l3:     li      t4, 176          
        beq     zero, zero, pro
l4:     li      t4, 153          
        beq     zero, zero, pro
l5:     li      t4, 146          
        beq     zero, zero, pro
l6:     li      t4, 130         
        beq     zero, zero, pro
l7:     li      t4, 248          
        beq     zero, zero, pro
l8:     li      t4, 128          
        beq     zero, zero, pro
l9:     li      t4, 152          
        beq     zero, zero, pro
la:     li      t4, 136          
        beq     zero, zero, pro
lb:     li      t4, 131          
        beq     zero, zero, pro
lc:     li      t4, 198          
        beq     zero, zero, pro
ld:     li      t4, 161          
        beq     zero, zero, pro
le:     li      t4, 134          
        beq     zero, zero, pro
lf:     li      t4, 142          
        beq     zero, zero, pro



pro:    li      t2, 8
        srl     a1, a1, t2
        li      t2, 24
        sll     t4, t4, t2
        or      a1, a1, t4              # put new bits in current memory value
        
        
        addi    t3, t3, 1               # inc hex index
        and     t4, t3, t6              # check if this is memory-alighed address
        bne     t4, zero, cont          # if not alighed, do not write memory

        addi    t4, t3, -4              # compute effective memory address
        sub     t4, zero, t4            # change the sign

        sw      a1, 0x24 (t4)           # store value to hex memory
        mv      a1, zero                # reset the value

cont:   li      t2, 4
        srl     t0, t0, t2              # shift counter value by 4 bits
        bne     t3, t5, digit           # if no all hex are set, loop back

        lw      t0, 0x400 (zero)        # restore t0
        lw      t1, 0x404 (zero)        # restore t1
        lw      t2, 0x408 (zero)        # restore t2
        lw      t3, 0x40c (zero)        # restore t3
        lw      t4, 0x410 (zero)        # restore t4
        lw      t5, 0x414 (zero)        # restore t5
        lw      t6, 0x418 (zero)        # restore t6
        lw      a1, 0x41c (zero)        # restore a1
        beq     zero, zero, ret
