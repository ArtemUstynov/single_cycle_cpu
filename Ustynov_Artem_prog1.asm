.text                  # beginning of the text segment (or code segment)

beq $0, $0, start

swap: 
    addi $t8, $t4, 0 
    sw $t4, 0x4($t0)
    sw $t5, 0x0($t0)
    beq $zero, $zero, end_swap

sort: 
add $t0, $a0, $zero # pointer to array 
add $t1, $a1, $zero # size of the array 
addi $t2, $zero, 1  # var to go through loop_a 
addi $t7, $zero, 0  # var to go through loop_b
# t9 is for smaller then 
# t6 celling for loop_b
for_a:
    beq $t1, $t2, end_a # end loop_a 

    for_b:
    	sub $t6, $t1, $t2
    	sub $t6, $t6, $t7
        beq $zero, $t6, end_b # end loop_b
        
        lw $t4, 0x0($t0) # element of the array to be compared
        lw $t5, 0x4($t0) # arr j+1
        slt $t9, $t4, $t5
            beq $t9, $zero, swap
            end_swap:
	
	addi $t0, $t0, 0x4 
    	add $t7, $t7, 1  # increment loop_b cnt
       	beq $zero, $zero, for_b # jmp to start of loop
       	
    end_b:
    addi $t7, $zero, 0 
    addi $t0, $a0, 0 
    add $t2, $t2, 1  # increment loop_a cnt
    beq $zero, $zero, for_a # jmp to start of loop_a 
end_a:

jr $ra

start: 
# simulate data loaded
lw  $s0, 0xC($0)# first elem of array 
lw  $s1, 0x8($0)# number of elements

#go to function 
add   $a0, $s0, $zero	
add   $a1, $s1, $zero

#slt $t8, $zero, $s0 
#beq $t8, $zero, fin 
#slt $t8, $zero, $s1 
#beq $t8, $zero, fin 

jal sort
#fin:
addi $t7, $zero, 'e'
nop


