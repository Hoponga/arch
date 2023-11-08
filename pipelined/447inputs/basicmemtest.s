main: 
    addiu $t1 $zero 0x1000
    jal ending
    addiu $t2 $zero 3
    sll $t1 $t1 16
    addiu $3 $zero 150
    sw $3 0($t1) 
    sw $t2 8($t1)
    lw $4 0($t1) 
    lw $6 8($t1)
ending: 
    addiu   $v0, $zero, 0xa
    syscall

