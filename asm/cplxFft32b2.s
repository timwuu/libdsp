/**********************************************************************
* � 2006 Microchip Technology Inc.
*
* FileName:        cplxfft32b.s
* Dependencies:    Header (.h/.inc) files if applicable, see below
* Processor:       dsPIC33Fxxxx/PIC24Hxxxx
* Compiler:        MPLAB� C30 v2.01.00 or higher
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Inc. (�Microchip�) licenses this software to you
* solely for use with Microchip dsPIC� digital signal controller
* products. The software is owned by Microchip and is protected under
* applicable copyright laws.  All rights reserved.
*
* SOFTWARE IS PROVIDED �AS IS.�  MICROCHIP EXPRESSLY DISCLAIMS ANY
* WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
* PARTICULAR PURPOSE, OR NON-INFRINGEMENT. IN NO EVENT SHALL MICROCHIP
* BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL
* DAMAGES, LOST PROFITS OR LOST DATA, HARM TO YOUR EQUIPMENT, COST OF
* PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
* BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
* ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER SIMILAR COSTS.
*
* REVISION HISTORY:
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Author            Date      Comments on this revision
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* MCHP	          22Sep06  First release of source file
*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*
* ADDITIONAL NOTES:
*
**********************************************************************/


	; Local inclusions.
	.nolist
	.include	"dspcommon.inc"		; MODCON, XBREV
	.list

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	.section .libdsp, code
		
	.global _FFTComplex32bIP



	.equ	TF_FFT_SIZE, 1024	    ;1024			; 10stage, 1024 point complex FFT
	.equ	TF_FFT_STG, 10   	    ;10


	.text


;******************************************************************************
; Butterfly Computation
; P=Pr+jPi, Q=Qr+jQi, W=Wr-jWi
; P'=P+W*Q and Q'=P-W*Q
;
; Indirect Address Pointers, it will be update to point to next date inputs 
; W10->(Pr,Pi,Pr+1,Pi+1,....)
; W11->(Qr,Qi,Qr+1,Qi+1,....)
; W8-> Wr
; W9-> Wi

; W12 -> ACCA
; W13 -> ACCB
; W14 -> FP, First two location used for TEMP

; Equations for Butterfly Computation
; Pr'=(Pr+Qr*Wr+Qi*Wi)/2  
; Qr'=(Pr-Qr*Wr-Qi*Wi)/2
; Pi'=(Pi+Qi*Wr-Qr*Wi)/2
; Qi'=(Pi-Qi*WR+Qr*Wi)/2
;******************************************************************************          
	.macro BFLY_MACRO
; Qr*Wi 32-bit multiplication


			MOV.D	[W9],W6				; (w6,w7)=Wi=(y0,y1)


											; (W11,W11+1)=Wi=(x0,x1)
.ifndef __dsPIC33E

			MUL.SU	w7,[W11],w0				; y1*x0
			MUL.US 	w6,[++W11],w2			; y0*x1
			MUL.SS	w7,[W11++],w4			; y1*x1

			LAC		w3,B
			ADD		w1,B					
			SFTAC	B,#16					; B=(y1*x0 + y0*x1)>>16

			ADD	w4,[w13++],[w14++]
			ADDC	w5,[w13--],[w14--]		; TEMP=Qr*Wi=y1*x1 + (y1*x0 + y0*x1)>>16	

.else

			MOV [W11++],w4  ; w4=x0

			MPY W4*W7, B, [W11]+=2,W5
			MAC W5*W6, B, [W11]+=2,W4; Word0 (unsigned) x Word3 (signed)
			SFTAC B, #16; Shift right by 15 bits to align for Q31 format
			MAC W5*W7, B, [W11]-=6,W5; Word1 (signed) x Word 3 (signed)
			
			MOV [w13++],[w14++]
			MOV [w13--],[w14--]
.endif

; Qi*Wi 32-bit multiplication

.ifndef __dsPIC33E
											; (w6,w7)=Wi=(y0,y1)
											; (W11,W11+1)=Qi=(x0,x1)								
			MUL.SU	w7,[W11],w0				; y1*x0
			MUL.US 	w6,[++W11],w2			; y0*x1
			MUL.SS	w7,[W11--],w4			; y1*x1

			LAC		w3,B
			ADD		w1,B					
			SFTAC	B,#16					; B=(y1*x0 + y0*x1)>>16
		
			ADD 	w4,[w13],[w13++]
			ADDC	w5,[w13],[w13--]		; B=Qi*Wi=y1*x1 + (y1*x0 + y0*x1)>>16	

.else

			MPY W5*W6, B; Word1 (signed) x Word2 (unsigned)
			MAC W4*W7, B, [W11]+=2,W4; Word0 (unsigned) x Word3 (signed)
			SFTAC B, #16; Shift right by 15 bits to align for Q31 format
			MAC W5*W7, B, [W11]+=2,W5; Word1 (signed) x Word 3 (signed)
			
.endif

; Qr*Wr 32-bit multiplication
			MOV.D	[W8],W6				; (w6,w7)=Wr=(y0,y1)

.ifndef __dsPIC33E			

			MUL.US 	w6,[--W11],w2			; y0*x1
			MUL.SS	w7,[W11--],w4			; y1*x1
			MUL.SU	w7,[W11],w0				; y1*x0

			LAC		w3,A
			ADD		w1,A					
			SFTAC	A,#16					; A=(y1*x0 + y0*x1)>>16
		
			ADD 	w4,[w12],[w12++]
			ADDC	w5,[w12],[w12--]		; A=Qr*Wr=y1*x1 + (y1*x0 + y0*x1)>>16

.else

			MPY W5*W6, A; Word1 (signed) x Word2 (unsigned)
			MAC W4*W7, A, [W11]+=2,W4; Word0 (unsigned) x Word3 (signed)
			SFTAC A, #16; Shift right by 15 bits to align for Q31 format
			MAC W5*W7, A, [W11]-=6,W5; Word1 (signed) x Word 3 (signed)

.endif

; Pr'=(Pr+Qr*Wr+Qi*Wi)/2  
; Qr'=(Pr-Qr*Wr-Qi*Wi)/2		
			ADD		A						; A=Qr*Wr+Qi*Wi		
			LAC		[++W10],B				; B=Pr	
			MOV		[--W10],[w13]
			SFTAC	B,#1					; B=Pr/2 in Q31 format
			ADD		A						; A=(Pr + Qr*Wr+Qi*Wi)/2 in Q31 format
			
			SFTAC	B,#-1
			SUB		B						; B=(Pr - Qr*Wr-Qi*Wi)/2 in Q31 format
		
			MOV		[w12++],[W10++]
			MOV		[w12--],[W10++]			; Pr'=(Pr+Qr*Wr+Qi*Wi)/2  
		
			MOV		[w13++],[W11++]
			MOV		[w13--],[W11++]			; Qr'=(Pr-Qr*Wr-Qi*Wi)/2  	
	
	
; Qi*Wr 32-bit multiplication
.ifndef __dsPIC33E			
											; (w6,w7)=Wr=(y0,y1)
											; (W11,W11++)=Qi=(x0,x1)
			MUL.SU	w7,[W11],w0				; y1*x0
			MUL.US 	w6,[++W11],w2			; y0*x1
			MUL.SS	w7,[W11--],w4			; y1*x1

			LAC		w3,A
			ADD		w1,A					
			SFTAC	A,#16					; B=(y1*x0 + y0*x1)>>16
		
			ADD 	w4,[w12],[w12++]
			ADDC	w5,[w12],[w12--]		; A=Qi*Wr=y1*x1 + (y1*x0 + y0*x1)>>16	

.else

			MPY W5*W6, A; Word1 (signed) x Word2 (unsigned)
			MAC W4*W7, A; Word0 (unsigned) x Word3 (signed)
			SFTAC A, #16; Shift right by 15 bits to align for Q31 format
			MAC W5*W7, A; Word1 (signed) x Word 3 (signed)

.endif			
; Pi'=(Pi+Qi*Wr-Qr*Wi)/2
; Qi'=(Pi-Qi*Wr+Qr*Wi)/2
			LAC		[++W14],B					
			MOV		[--w14],[w13]			; B=Qr*Wi
	
			SUB		A						; A=Qi*Wr-Qr*Wi
			LAC		[++W10],B				; B=Pr	
			MOV		[--W10],[w13]
			SFTAC	B,#1					; B=Pi/2 in Q31 format
			ADD		A						; A=(Pi + Qi*Wr-Qr*Wi)/2 in Q31 format
			
			SFTAC	B,#-1
			SUB		B						; B=(Pi - Qi*Wr+Qr*Wi)/2 in Q31 format
		
			MOV		[w12++],[W10++]
			MOV		[w12--],[W10++]			; Pi'=(Pi + Qi*Wr-Qr*Wi)/2 
		
			MOV		[w13++],[W11++]
			MOV		[w13--],[W11++]			; Qi'=(Pi - Qi*Wr+Qr*Wi)/2  	

	.endm

.ifdef __dsPIC33E
	.macro BFLY_MACRO_PSV
; Qr*Wi 32-bit multiplication


			POP		DSRPAG
			MOV.D	[W9],W6				; (w6,w7)=Wi=(y0,y1)


											; (W11,W11+1)=Wi=(x0,x1)
			PUSH	DSRPAG
			MOVPAG  #0x01,DSRPAG

			MOV [W11++],w4  ; w4=x0

	    
			;W0020-CORE: Trap due to memory access outside X data space, occurred from instruction at 0x001154.
			MPY W4*W7, B, [W11]+=2,W5  ;timijk 
			MAC W5*W6, B, [W11]+=2,W4; Word0 (unsigned) x Word3 (signed)
			SFTAC B, #16; Shift right by 15 bits to align for Q31 format
			MAC W5*W7, B, [W11]-=6,W5; Word1 (signed) x Word 3 (signed)
			
			MOV [w13++],[w14++]
			MOV [w13--],[w14--]

; Qi*Wi 32-bit multiplication
			MPY W5*W6, B; Word1 (signed) x Word2 (unsigned)
			MAC W4*W7, B, [W11]+=2,W4; Word0 (unsigned) x Word3 (signed)
			SFTAC B, #16; Shift right by 15 bits to align for Q31 format
			MAC W5*W7, B, [W11]+=2,W5; Word1 (signed) x Word 3 (signed)
			
; Qr*Wr 32-bit multiplication
			POP		DSRPAG									
			MOV.D	[W8],W6				; (w6,w7)=Wr=(y0,y1)

											; (W11-2,W11-1)=Qr=(x0,x1)
			PUSH	DSRPAG
			MOVPAG  #0x01,DSRPAG
								
			MPY W5*W6, A; Word1 (signed) x Word2 (unsigned)
			MAC W4*W7, A, [W11]+=2,W4; Word0 (unsigned) x Word3 (signed)
			SFTAC A, #16; Shift right by 15 bits to align for Q31 format
			MAC W5*W7, A, [W11]-=6,W5; Word1 (signed) x Word 3 (signed)

; Pr'=(Pr+Qr*Wr+Qi*Wi)/2  
; Qr'=(Pr-Qr*Wr-Qi*Wi)/2		
			ADD		A						; A=Qr*Wr+Qi*Wi		
			LAC		[++W10],B				; B=Pr	
			MOV		[--W10],[w13]
			SFTAC	B,#1					; B=Pr/2 in Q31 format
			ADD		A						; A=(Pr + Qr*Wr+Qi*Wi)/2 in Q31 format
			
			SFTAC	B,#-1
			SUB		B						; B=(Pr - Qr*Wr-Qi*Wi)/2 in Q31 format
		
			MOV		[w12++],[W10++]
			MOV		[w12--],[W10++]			; Pr'=(Pr+Qr*Wr+Qi*Wi)/2  
		
			MOV		[w13++],[W11++]
			MOV		[w13--],[W11++]			; Qr'=(Pr-Qr*Wr-Qi*Wi)/2  	
	
	
; Qi*Wr 32-bit multiplication

			MPY W5*W6, A; Word1 (signed) x Word2 (unsigned)
			MAC W4*W7, A; Word0 (unsigned) x Word3 (signed)
			SFTAC A, #16; Shift right by 15 bits to align for Q31 format
			MAC W5*W7, A; Word1 (signed) x Word 3 (signed)

; Pi'=(Pi+Qi*Wr-Qr*Wi)/2
; Qi'=(Pi-Qi*Wr+Qr*Wi)/2
			LAC		[++W14],B					
			MOV		[--w14],[w13]			; B=Qr*Wi
	
			SUB		A						; A=Qi*Wr-Qr*Wi
			LAC		[++W10],B				; B=Pr	
			MOV		[--W10],[w13]
			SFTAC	B,#1					; B=Pi/2 in Q31 format
			ADD		A						; A=(Pi + Qi*Wr-Qr*Wi)/2 in Q31 format
			
			SFTAC	B,#-1
			SUB		B						; B=(Pi - Qi*Wr+Qr*Wi)/2 in Q31 format
		
			MOV		[w12++],[W10++]
			MOV		[w12--],[W10++]			; Pi'=(Pi + Qi*Wr-Qr*Wi)/2 
		
			MOV		[w13++],[W11++]
			MOV		[w13--],[W11++]			; Qi'=(Pi - Qi*Wr+Qr*Wi)/2  	

	.endm
.endif


;............................................................................
; Pointer Usage
; W10->(Pr1,Pi1)
; W11->(Pr2,Pi2)
; W8->(Pr3,Pi3)
; W9->(Pr4,Pi4)
;
; W12 -> ACCA
; W13 -> ACCB
; W14 -> FP, First two location used for TEMP
;
; Calculations
; Pr6   ->      Pr1'=(Pr1+Pr2+Pr3+Pr4)/4
; Pr2   ->      Pr2'=(Pr1-Pr2+Pi3-Pi4)/4
; Pr3   ->      Pr3'=(Pr1+Pr2-Pr3-Pr4)/4
; Pr4   ->      Pr4'=(Pr1-Pr2-Pi3+Pi4)/4
; Pr6+1 ->      Pi1'=(Pi1+Pi2+Pi3+Pi4)/4
; Pr2+1 ->      Pi2'=(Pi1-Pi2-Pr3+Pr4)/4
; Pr3+1 ->      Pi3'=(Pi1+Pi2-Pi3-Pi4)/4
; Pr4+1 ->      Pi4'=(Pi1-Pi2+Pr3-Pr4)/4
;............................................................................ 
   

	.macro STG12_MACRO
;----------
			LAC		[++W10],B				; B=Pr1	
			MOV		[--W10],[w13]
			SFTAC	B,#2					; B=Pr1/4 in Q31 format
			
			LAC		[++W11],A				; A=Pr2	
			MOV		[--W11],[w12]
			SFTAC	A,#2					; A=Pr2/4 in Q31 format

			ADD		A						; A=(Pr1+Pr2)/4
	
			SFTAC	B,#-1
			SUB		B						; B=(Pr1-Pr2)/4

			MOV		[w12++],[W10++]			; Pr1=(Pr1+Pr2)/4
			MOV		[w12--],[W10++]			

			MOV		[w13++],[W11++]			; Pr2=(Pr1-Pr2)/4
			MOV		[w13--],[W11++]			  

;----------
			LAC		[++W8],B				; B=Pr3	
			MOV		[--W8],[w13]
			SFTAC	B,#2					; B=Pr3/4 in Q31 format
			
			LAC		[++W9],A				; A=Pr4	
			MOV		[--W9],[w12]
			SFTAC	A,#2					; A=Pr4/4 in Q31 format

			ADD		A						; A=(Pr3+Pr4)/4
	
			SFTAC	B,#-1
			SUB		B						; B=(Pr3-Pr4)/4

			MOV		[w12++],[W8++]			; Pr3=(Pr3+Pr4)/4
			MOV		[w12--],[W8++]			

			MOV		[w13++],[W9++]			; Pr4=(Pr3-Pr4)/4
			MOV		[w13--],[W9++]		

			MOV		[w13++],w0				; (w0,w1)=(Pr3-Pr4)/4
			MOV		[w13--],w1			  

;----------
			LAC		[++W10],B				; B=Pi1	
			MOV		[--W10],[w13]
			SFTAC	B,#2					; B=Pi1/4 in Q31 format
			
			LAC		[++W11],A				; A=Pi2	
			MOV		[--W11],[w12]
			SFTAC	A,#2					; A=Pi2/4 in Q31 format

			ADD		A						; A=(Pi1+Pi2)/4
	
			SFTAC	B,#-1
			SUB		B						; B=(Pi1-Pi2)/4

			MOV		[w12++],[W10++]			; Pi1=(Pi1+Pi2)/4
			MOV		[w12--],[W10--]			

			MOV		[w13++],[W11++]			; Pi2=(Pi1-Pi2)/4
			MOV		[w13--],[W11--]			  

;----------
			LAC		[++W8],B				; B=Pi3	
			MOV		[--W8],[w13]
			SFTAC	B,#2					; B=Pi3/4 in Q31 format
			
			LAC		[++W9],A				; A=Pi4	
			MOV		[--W9],[w12]
			SFTAC	A,#2					; A=Pi4/4 in Q31 format

			ADD		A						; A=(Pi3+Pi4)/4
	
			SFTAC	B,#-1
			SUB		B						; B=(Pi3-Pi4)/4

			MOV		[w12++],[W8++]			; Pr3=(Pi3+Pi4)/4
			MOV		[w12--],[W8--]			

			MOV		[w13++],w2				; (w2,w3)=(Pi3-Pi4)/4
			MOV		[w13--],w3				
;---------------------------------
;---------------------------------	  
;---------- (Pi1,Pi3)
			LAC		[++W10],B				; B=(Pi1+Pi2)/4	
			MOV		[--W10],[w13]
			
			LAC		[++W8],A				; A=(Pi3+Pi4)/4	
			MOV		[--W8],[w12]

			ADD		A						; A=(Pi1+Pi2+Pi3+Pi4)/4
	
			SFTAC	B,#-1
			SUB		B						; B=(Pi1+Pi2-Pi3-Pi4)/4

			MOV		[w12++],[W10++]			; Pi1=(Pi1+Pi2+Pi3+Pi4)/4
			MOV		[w12--],[W10--]			

			MOV		[w13++],[W8++]			; Pi3=(Pi1+Pi2-Pi3-Pi4)/4
			MOV		[w13--],[W8--]			  

;---------- (Pr1,Pr3)
			LAC		[--W10],B				; B=(Pr1+Pr2)/4	
			MOV		[--W10],[w13]
			
			LAC		[--W8],A				; A=(Pr3+Pr4)/4	
			MOV		[--W8],[w12]

			ADD		A						; A=(Pr1+Pr2+Pr3+Pr4)/4
	
			SFTAC	B,#-1
			SUB		B						; B=(Pr1+Pr2-Pr3-Pr4)/4

			MOV		[w12++],[W10++]			; Pr1=(Pr1+Pr2+Pr3+Pr4)/4
			MOV		[w12--],[W10--]			

			MOV		[w13++],[W8++]			; Pr3=(Pr1+Pr2-Pr3-Pr4)/4
			MOV		[w13--],[W8--]			  

;---------- (Pi2,Pi4)
			LAC		[++W11],B				; B=(Pi1-Pi2)/4	
			MOV		[--W11],[w13]
			
			LAC		w1,A					; A=(Pr3-Pr4)/4	
			MOV		w0,[w12]

			ADD		A						; A=(Pi1-Pi2+Pr3-Pr4)/4
	
			SFTAC	B,#-1
			SUB		B						; B=(Pi1-Pi2-Pr3+Pr4)/4

			MOV		[w13++],[W11++]			; Pi2=(Pi1-Pi2-Pr3+Pr4)/4
			MOV		[w13--],[W11--]			

			MOV		[w12++],[W9++]			; Pi4=(Pi1-Pi2+Pr3-Pr4)/4
			MOV		[w12--],[W9--]			  

;----------(Pr2,Pr4)
			LAC		[--W11],B				; B=(Pr1-Pr2)/4	
			MOV		[--W11],[w13]
			
			LAC		w3,A					; A=(Pi3-Pi4)/4	
			MOV		w2,[w12]

			ADD		A						; A=(Pr1-Pr2+Pi3-Pi4)/4
	
			SFTAC	B,#-1
			SUB		B						; B=(Pr1-Pr2-Pi3+Pi4)/4

			MOV		[w12++],[W11++]			; Pr2=Pr1-Pr2+Pi3-Pi4)/4
			MOV		[w12--],[W11--]			

			MOV		[++w13],[--W9]			; Pr4=(Pr1-Pr2-Pi3+Pi4)/4
			MOV		[--w13],[--W9]			  	

	.endm

; Local Stack Frame
;............................................................................
;   |_______|
;   |_______|<- Stack Pointer                          	SP
;   |_______|<- Stack Pointer Return                   	SP+20
;   |_______|<- Buffer Pointer			                FP+18
;   |_______|<- Wr Pointer								FP+16
;   |_______|<- Wi Pointer                           	FP+14 
;   |_______|<- Stage Counter				            FP+12
;   |_______|<- Block Counter			                FP+10
;   |_______|<- Butterfly Counter		                FP+8
;   |_______|<- Block INDEX     						FP+6
;   |_______|<- Butterfly INDEX	              			FP+4
;   |_______|<- Temp                               		FP                                              
;............................................................................

    .equ    SP_RET,     20
	.equ	BUF_PTR,	18
	.equ	WR_PTR,		16
	.equ	WI_PTR,		14
	.equ	STG_CNTR,   12
	.equ 	BLK_CNTR,   10
	.equ	BFL_CNTR,   8
	.equ 	BLK_INDX, 	6
	.equ	BFL_INDX,  	4
	.equ	TEMP,		0
 

;............................................................................
; Complex FFT Calculation
;............................................................................
_FFTComplex32bIP:
;............................................................................
; Context Save
			PUSH.D	W10						; {W10,W11} to TOS
			PUSH.D	W8						; {W8,W9} to TOS
			PUSH.D	w12						; {w12,w13} to TOS
			PUSH	CORCON
	.ifdef __dsPIC33E
			PUSH 	DSRPAG
	.else
			PUSH	PSVPAG
	.endif
			LNK     #22
;............................................................................
;;	PSVPAG = __builtin_psvpage(&twiddleFactor);
;;	CORCONbits.PSV = 1;	
.ifdef __dsPIC33E
			BSET	CORCON,#0           ; Set IF bit
			BSET	CORCON,#13          ; Set mixed-sign mode
			BCLR	CORCON,#12

        	MOV	    #COEFFS_IN_DATA,W7	; W7 = COEFFS_IN_DATA
	        CP	    W7,W3				; W7 - W3
	        BRA	    Z,_NOPSV            ; IF W4 = COEFFS_IN_DATA
		NOP

        	; TWIDDLES IN FLASH
		MOVPAG W3,DSRPAG            ; SET DSRPAG TO PSV PAGE
		CP     W0,#6                ; check if FFTSIZE<=128
		BRA    LEU,_RUNFROMFLASH

         	; CHECK IF SUFFICIENT SPACE IS AVAILABLE IN STACK
        	; CONDITION (SP + TABLE SIZE + STACK_GUARD) < SPLIM
	        MOV    SPLIM,W5             ; LOAD STACK POINTER LIMIT
          	MOV    _STACK_GUARD,W13     ; STACK GUARD SPACE FOR INTERRUPTS ETC ...
	                            ; NOTE: THIS IS USER-DEFINABLE
		MOV    #__YDATA_BASE,W4

		MOV	   #TF_FFT_SIZE,W11		; 
          	SL     W11,W12               ; 
          	ADD    W11,W12,W11            ; TABLE_SIZE IN BYTES

	        ADD    W11,W15,W8           ; SP + TABLE_SIZE
	        BRA    c,_RUNFROMFLASH
	        SUB    W8,W4,W8           ; SP + TABLE_SIZE - __YDATA_BASE
	        BRA    geu,_RUNFROMFLASH
	
          	ADD    W11,W13,W11            ; ADD _STACK_GUARD
           	BRA    C,_RUNFROMFLASH
	        ADD    W11,W15,W11            ; ADD SP
         	BRA    C,_RUNFROMFLASH

        	SUB    W11,W5,W13            ; CHECK AGAINST SPLIM
        	BRA    LTU,_COPYTOSTACK

_RUNFROMFLASH:
			PUSH	DSRPAG
			MOVPAG	#0x01,DSRPAG

; Store Input Parameters
			MOV		w1,[w14+BUF_PTR]	
			MOV		w2,[w14+WI_PTR]			; Wi = SIN(*)
			MOV		#TF_FFT_SIZE,W10
			ADD		w2,W10,w2
			MOV		w2,[W14+WR_PTR]			; Wr = COS(*)
			MOV		w0,[w14+STG_CNTR]
	

			MOV		#ACCAL,w12
			MOV		#ACCBL,w13

; Stage 1 to 2 unrolled for cycle efficiency
;............................................................................ 
			MOV		w1,W10
			ADD		w1,#8,W11
			ADD		w1,#16,W8
			ADD		w1,#24,W9
                                                                                        
     
			MOV		#1,w2		
			SL		w2,w0,w2				; N = 1<<logN2
			LSR		w2,#2,w2
			DEC		w2,w2

			MOV     #32,w0        			; w0=32
			MOV		w0,[w14+BFL_INDX]  

; Loop over butterflies in a block
;............................................................................
			DO      w2,STG12_END_PSV
STG12_START_PSV:  

			STG12_MACRO
			MOV		[w14+BFL_INDX],w0
			ADD		W10,w0,W10				; Next Data Set
			ADD		W11,w0,W11				; Next Data Set
			ADD		W8,w0,W8				; Next Data Set
STG12_END_PSV:	ADD		W9,w0,W9				; Next Data Set
 ;............................................................................


; Stage 3 to LOG2N 
;............................................................................ 
; Butterfly counter for each block, Block counter for each stage and Stage Counter
; Block Index between P & Q inputs of Butterfly for Stage 3
; Twiddle Index for stage 3  
	
			MOV		#4,w0
			MOV		w0,[w14+BFL_CNTR]		; BFL_CNTR=4
			MOV		[w14+STG_CNTR],w1
			MOV		#1,w2		
			SL		w2,w1,w2				; N = 1<<logN2
			LSR		w2,#3,w2				; N/8
			MOV		w2,[w14+BLK_CNTR]		; BLK_CNTR=N/8	
		
			MOV		#(TF_FFT_SIZE/2),w2		
			MOV		w2,[w14+BFL_INDX]

			MOV		#32,w0
			MOV		w0,[w14+BLK_INDX]

			SUB		w1,#2,w1				; STG_CNTR=STG_CNTR-1
			MOV		w1,[w14+STG_CNTR]	


; Stage 1 to LOG2N 
;............................................................................ 
;			MOV		#1,w0
;			MOV		w0,[w14+BFL_CNTR]		; BFL_CNTR=1
;			MOV		[w14+STG_CNTR],w1
;			MOV		#1,w2		
;			SL		w2,w1,w2				; N = 1<<logN2
;			LSR		w2,#1,w2				; N/2
;			MOV		w2,[w14+BLK_CNTR]		; BLK_CNTR=N/2
;
;			MOV		#(TF_FFT_SIZE*2),w2		
;			MOV		w2,[w14+BFL_INDX]
;
;			MOV		#8,w0
;			MOV		w0,[w14+BLK_INDX]

	
; Loop over stages
;............................................................................
STG_START_PSV:                        
			MOV		[w14+BUF_PTR],W10		; Next Butterfly Block (Pr)
			MOV		[w14+BLK_INDX],W11		;timijk:debug
			ADD		W10,W11,W11				; Next Butterfly Block (Qr)


			MOV		[w14+BLK_CNTR],w0		; Block loop count
			DEC 	w0,w0


; Loop over blocks in a stage
;............................................................................
			DO      w0,BLK_END_PSV
.ifdef __dsPIC33E
			NOP
.endif
BLK_START_PSV:	
			MOV		[w14+WR_PTR],W8		; Initialise Twiddle Factor (Wr) pointer
			MOV		[w14+WI_PTR],W9		; Initialisae Twiddle Factor (Wi) pointer
			MOV		[w14+BFL_CNTR],w0		; Butterfly loop count
			DEC 	w0,w0
; Loop over butterflies in a block
;............................................................................
			DO      w0,BFL_END_PSV
BFL_START_PSV:  
			BFLY_MACRO_PSV						;timijk
			MOV		[w14+BFL_INDX],w0
			ADD		W8,w0,W8				; Next Twiddle Factor (Wr)
BFL_END_PSV:		ADD		W9,w0,W9				; Next Twiddle Factor (Wi)
;............................................................................            
			MOV		[w14+BLK_INDX],w0
			ADD		W10,w0,W10				; Next Butterfly Block (Pr)
BLK_END_PSV:		ADD		W11,w0,W11				; Next Butterfly Block(Qr)
 
;............................................................................      
			ADD		W14,#BFL_INDX,W0


			LAC		[w0],#1,A				; BFL_INDX = BFL_INDX/2
			SAC		A,[w0++]

			LAC		[w0],#-1,A				; BLK_INDX = BLK_INDX * 2
			SAC		A,[w0++]	



			LAC		[w0],#-1,A				; BFL_CNTR = BFL_CNTR * 2
			SAC		A,[w0++]	

			LAC		[w0],#1,A				; BLK_CNTR = BLK_CNTR/2
			SAC		A,[w0++]
			
			DEC		[w0],[w0]				; STG_CNTR = STG_CNTR - 1
			BRA     NZ,STG_START_PSV      
			
			POP     DSRPAG
			BRA     _DONECPLXFFT

_COPYTOSTACK:
            ; save SP_RET
			MOV		W15,[W14+SP_RET]
			MOV     W15,W8
			
        	MOV	   #TF_FFT_SIZE,W11		; 
	        SL     W11,W12               ; 
         	ADD    W11,W12,W11            ; TABLE_SIZE IN WORDS
			
        	; COPY ENTIRE TWIDDLE TABLE TO STACK
	        DEC    W11,W13           ; REPEAT COUNT
	        REPEAT W13              ; SET REPEAT COUNT
	        MOV    [W2++],[w15++]   ; copy from PSV to stack
	
; Store Input Parameters
			MOV		w1,[w14+BUF_PTR]	
			MOV		W8,[w14+WI_PTR]			; Wi = SIN(*)
			MOV		#TF_FFT_SIZE,W10
			ADD		W8,W10,W8
			MOV		W8,[W14+WR_PTR]			; Wr = COS(*)
			MOV		w0,[w14+STG_CNTR]

			MOVPAG	#0x01,DSRPAG
			NOP
			BRA     _INPUTPARAMSSTORED

_NOPSV:
			MOV		W15,[W14+SP_RET]          
			MOVPAG	#0x01,DSRPAG
			NOP
.else	
			MOV		w3,PSVPAG		
			BSET	CORCON,#2
.endif

; Store Input Parameters
			MOV		w1,[w14+BUF_PTR]	
			MOV		w2,[w14+WI_PTR]			; Wi = SIN(*)
			MOV		#TF_FFT_SIZE,W10
			ADD		w2,W10,w2
			MOV		w2,[W14+WR_PTR]			; Wr = COS(*)
			MOV		w0,[w14+STG_CNTR]
	
_INPUTPARAMSSTORED:
			MOV		#ACCAL,w12
			MOV		#ACCBL,w13

; Stage 1 to 2 unrolled for cycle efficiency
;............................................................................ 
			MOV		w1,W10
			ADD		w1,#8,W11
			ADD		w1,#16,W8
			ADD		w1,#24,W9
                                                                                        
     
			MOV		#1,w2		
			SL		w2,w0,w2				; N = 1<<logN2
			LSR		w2,#2,w2
			DEC		w2,w2

            MOV     #32,w0        			; w0=32
			MOV		w0,[w14+BFL_INDX]  

; Loop over butterflies in a block
;............................................................................
			DO      w2,STG12_END         
STG12_START:  

			STG12_MACRO
			MOV		[w14+BFL_INDX],w0
			ADD		W10,w0,W10				; Next Data Set
			ADD		W11,w0,W11				; Next Data Set
			ADD		W8,w0,W8				; Next Data Set
STG12_END:	ADD		W9,w0,W9				; Next Data Set
 ;............................................................................


; Stage 3 to LOG2N 
;............................................................................ 
; Butterfly counter for each block, Block counter for each stage and Stage Counter
; Block Index between P & Q inputs of Butterfly for Stage 3
; Twiddle Index for stage 3  
	
			MOV		#4,w0
			MOV		w0,[w14+BFL_CNTR]		; BFL_CNTR=4
			MOV		[w14+STG_CNTR],w1
			MOV		#1,w2		
			SL		w2,w1,w2				; N = 1<<logN2
			LSR		w2,#3,w2				; N/8
			MOV		w2,[w14+BLK_CNTR]		; BLK_CNTR=N/8	
		
			MOV		#(TF_FFT_SIZE/2),w2		
			MOV		w2,[w14+BFL_INDX]

			MOV		#32,w0
			MOV		w0,[w14+BLK_INDX]

			SUB		w1,#2,w1				; STG_CNTR=STG_CNTR-1
			MOV		w1,[w14+STG_CNTR]	


; Stage 1 to LOG2N 
;............................................................................ 
;			MOV		#1,w0
;			MOV		w0,[w14+BFL_CNTR]		; BFL_CNTR=1
;			MOV		[w14+STG_CNTR],w1
;			MOV		#1,w2		
;			SL		w2,w1,w2				; N = 1<<logN2
;			LSR		w2,#1,w2				; N/2
;			MOV		w2,[w14+BLK_CNTR]		; BLK_CNTR=N/2
;
;			MOV		#(TF_FFT_SIZE*2),w2		
;			MOV		w2,[w14+BFL_INDX]
;
;			MOV		#8,w0
;			MOV		w0,[w14+BLK_INDX]

	
; Loop over stages
;............................................................................
STG_START:                        
			MOV		[w14+BUF_PTR],W10		; Next Butterfly Block (Pr)
			MOV		[w14+BLK_INDX],W11	
			ADD		W10,W11,W11				; Next Butterfly Block (Qr)


			MOV		[w14+BLK_CNTR],w0		; Block loop count
			DEC 	w0,w0


; Loop over blocks in a stage
;............................................................................
			DO      w0,BLK_END
.ifdef __dsPIC33E
            NOP
.endif
BLK_START:	
			MOV		[w14+WR_PTR],W8		; Initialise Twiddle Factor (Wr) pointer
			MOV		[w14+WI_PTR],W9		; Initialisae Twiddle Factor (Wi) pointer
			MOV		[w14+BFL_CNTR],w0		; Butterfly loop count
			DEC 	w0,w0
; Loop over butterflies in a block
;............................................................................
			DO      w0,BFL_END       
BFL_START:  
			BFLY_MACRO
			MOV		[w14+BFL_INDX],w0
			ADD		W8,w0,W8				; Next Twiddle Factor (Wr)
BFL_END:	ADD		W9,w0,W9				; Next Twiddle Factor (Wi)
;............................................................................            
			MOV		[w14+BLK_INDX],w0
			ADD		W10,w0,W10				; Next Butterfly Block (Pr)
BLK_END:	ADD		W11,w0,W11				; Next Butterfly Block(Qr)
 
;............................................................................      
			ADD		W14,#BFL_INDX,W0


			LAC		[w0],#1,A				; BFL_INDX = BFL_INDX/2
			SAC		A,[w0++]

			LAC		[w0],#-1,A				; BLK_INDX = BLK_INDX * 2
			SAC		A,[w0++]	



			LAC		[w0],#-1,A				; BFL_CNTR = BFL_CNTR * 2
			SAC		A,[w0++]	

			LAC		[w0],#1,A				; BLK_CNTR = BLK_CNTR/2
			SAC		A,[w0++]
			
			DEC		[w0],[w0]				; STG_CNTR = STG_CNTR - 1
			BRA     NZ,STG_START        

			MOV		[W14+SP_RET],W15

_DONECPLXFFT:			
;............................................................................
; Context Restore
			ULNK
	.ifdef __dsPIC33E
			POP 	DSRPAG
	.else
			POP		PSVPAG
	.endif			

			POP		CORCON
			POP.D	w12						; {w12,w13} from TOS
			POP.D	W8						; {W8,W9} from TOS
			POP.D	W10						; {W10,W11} from TOS
;............................................................................
			RETURN	

