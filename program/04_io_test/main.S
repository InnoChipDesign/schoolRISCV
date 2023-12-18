        .text

init:   li      a2, 1024       # max value
        li      a1, 1          # min value
        li      t3, 1

loop:   mv      t0, a1
        lw      t2, 0x8 (zero)
        nop
        nop
        nop
        and     t1, t2, t3
        beq     t1, zero, skip
        li      t4, 2
        and     t4, t4, t2
        bne     t4, zero, l1
        li      t4, 1
        
l1:     mv      t1, a2
        srl     t1, t1, t4
        beq     zero, zero, fwd
skip:
        mv      t1, zero

fwd:    or      t2, t0, t1
        sw      t2, 0x0 (zero)
        add     t0, t0, t0
        beq     t1, a1, bcwd
        srl     t1, t1, a1
        bne     t0, a2, fwd
        
bcwd:   srl     t0, t0, a1
        or      t2, t0, t1
        sw      t2, 0x0 (zero)
        add     t1, t1, t1
        bne     t0, a1, bcwd

        beq     zero, zero, loop
