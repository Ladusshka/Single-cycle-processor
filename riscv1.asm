
lw t0, 4(x0) # array size  
lw t1, 8(x0) # array start address 
  
loop: 
    beq x0, t0, end # no elements left 
     
    lw  a0, 0(t1)   # load element to a0 
    jal ra, log     # jump to routine 
     
    sw  a0, 0(t1) 
     
    addi t1, t1, 4 
    addi t0, t0, -1 
     
    jal x0, loop 
 
end: 
    jal x0, end 
 
log:    # take exponent and convert to 2's complement 
    # floor_log t3, a0 # t3 <= floor_log(a0) 
     add a0, a0, t3   # a0 <= t3[31] ? D(a0) : a0 
    jalr  x0, ra, 0
		     






	
