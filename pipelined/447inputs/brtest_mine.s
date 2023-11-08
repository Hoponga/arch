main: 
addiu $t1 $x0 1 
addiu $t2 $t1 2 
addiu $t3 $x0 3
beq $t2 $t3 offset 
sub $t5 $t3 $t2 
offset: 
addiu   $v0, $zero, 0xa
   syscall