main: 
addiu $2 $zero 10000008
addiu $3 $zero 5
lw $4 4($2) 
addiu   $v0, $zero, 0xa
        syscall

