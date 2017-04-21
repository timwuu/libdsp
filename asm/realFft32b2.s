/**********************************************************************
* © 2006 Microchip Technology Inc.
*
* FileName:        realFFT.s
* Dependencies:    Header (.h/.inc) files if applicable, see below
* Processor:       dsPIC33Fxxxx/PIC24Hxxxx
* Compiler:        MPLAB® C30 v2.01.00 or higher
*
* SOFTWARE LICENSE AGREEMENT:
* Microchip Technology Inc. (“Microchip”) licenses this software to you
* solely for use with Microchip dsPIC® digital signal controller
* products. The software is owned by Microchip and is protected under
* applicable copyright laws.  All rights reserved.
*
* SOFTWARE IS PROVIDED “AS IS.”  MICROCHIP EXPRESSLY DISCLAIMS ANY
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
		
	.global _realFft32bIP


	.equ	TF_FFT_SIZE,1024			; 10stage, 1024 point complex FFT
	.equ	TF_FFT_STG, 10


	.text

;............................................................................
; MERGE ODD/EVEN FFT RESULTS
; W10->(Radd,Isub)
; W11->(Rsub,Iadd)
; W8-> Wr
; W9-> Wi

; W12 -> ACCA
; W13 -> ACCB
; W14 -> FP, First two location used for TEMP

; Equations for Butterfly Computation
; Gr(k)=Radd + (Wr*Iadd - Wi*Rsub)  
; Gi(k)=Isub - (Wr*Rsub + Wi*Iadd)
; Gr(N-k)=Radd - (Wr*Iadd - Wi*Rsub)  
; Gi(N-k)=-Isub - (Wr*Rsub + Wi*Iadd)        
;............................................................................

		.macro MERGE_BFLY_MACRO


; Iadd*Wi 32-bit multiplication
			MOV.D	[W9],W6				; (w6,w7)=Wi=(y0,y1)
											; (W11,W11+1)=Iadd=(x0,x1)
.ifndef __dsPIC33E

			MUL.SU	w7,[W11],w0				; y1*x0
			MUL.US 	w6,[++W11],w2			; y0*x1
			MUL.SS	w7,[W11--],w4			; y1*x1

			LAC		w3,B
			ADD		w1,B					
			SFTAC	B,#16					; B=(y1*x0 + y0*x1)>>16

			ADD		w4,[w13++],[w14++]
			ADDC	w5,[w13--],[w14--]		; TEMP=Iadd*Wi=y1*x1 + (y1*x0 + y0*x1)>>16	
.else

            MOV [W11++],w4  ; w4=x0

			MPY W4*W7, B, [W11]-=6,W5
            MAC W5*W6, B, [W11]+=2,W4; Word0 (unsigned) x Word3 (signed)
			SFTAC B, #16; Shift right by 15 bits to align for Q31 format
			MAC W5*W7, B, [W11]+=2,W5; Word1 (signed) x Word 3 (signed)

			MOV [w13++],[w14++]
			MOV [w13--],[w14--]
.endif

; Rsub*Wi 32-bit multiplication
											; (W11-2,W11-1)=Rsub=(x0,x1)
											; (w6,w7)=Wi=(x0,x1)								
.ifndef __dsPIC33E

			MUL.US 	w6,[--W11],w2			; y0*x1
			MUL.SS	w7,[W11--],w4			; y1*x1
			MUL.SU	w7,[W11++],w0			; y1*x0

			LAC		w3,B
			ADD		w1,B					
			SFTAC	B,#16					; B=(y1*x0 + y0*x1)>>16
		
			ADD 	w4,[w13],[w13++]
			ADDC	w5,[w13],[w13--]		; B=Rsub*Wi=y1*x1 + (y1*x0 + y0*x1)>>16	

.else

            MPY W5*W6, B; Word1 (signed) x Word2 (unsigned)
			MAC W4*W7, B, [W11]+=2,W4; Word0 (unsigned) x Word3 (signed)
			SFTAC B, #16; Shift right by 15 bits to align for Q31 format
			MAC W5*W7, B, [W11]-=6,W5; Word1 (signed) x Word 3 (signed)
			
.endif

; Iadd*Wr 32-bit multiplication		
			MOV.D	[W8],W6				; (w6,w7)=Wr=(y0,y1)
											; (W11+1,W11+2)=Rsub=(x0,x1)	
							
.ifndef __dsPIC33E
			MUL.SU	w7,[++W11],w0			; y1*x0
			MUL.US 	w6,[++W11],w2			; y0*x1
			MUL.SS	w7,[W11--],w4			; y1*x1

			LAC		w3,A
			ADD		w1,A					
			SFTAC	A,#16					; A=(y1*x0 + y0*x1)>>16
		
			ADD 	w4,[w12],[w12++]
			ADDC	w5,[w12],[w12--]		; A=Iadd*Wr=y1*x1 + (y1*x0 + y0*x1)>>16	

.else

            MPY W5*W6, A; Word1 (signed) x Word2 (unsigned)
			MAC W4*W7, A, [W11]+=2,W4; Word0 (unsigned) x Word3 (signed)
			SFTAC A, #16; Shift right by 15 bits to align for Q31 format
			MAC W5*W7, A, [W11]-=2,W5; Word1 (signed) x Word 3 (signed)
			
.endif

; Gr(k)=Radd + (Wr*Iadd - Wi*Rsub)  
; Gr(N-k)=Radd - (Wr*Iadd - Wi*Rsub)  	
			SUB		A						; A=Iadd*Wr-Rsub*Wi		
			LAC		[++W10],B				; B=Radd	
			MOV		[--W10],[w13]
			SFTAC	B,#1					; B=Radd/2 in Q31 format
			ADD		A						; A=(Radd + (Iadd*Wr-Rsub*Wi))/2 in Q31 format
			
			SFTAC	B,#-1
			SUB		B						; B=(Radd - (Iadd*Wr-Rsub*Wi))/2 in Q31 format
		
			MOV		[w12++],[W10++]
			MOV		[w12--],[W10++]			; Gr(k)=(Radd + (Iadd*Wr-Rsub*Wi))/2  
	
	
; Rsub*Wr 32-bit multiplication
											; (w6,w7)=Wr=(y0,y1)
											; (W11-2,W11-1)=Rsub=(x0,x1)
.ifndef __dsPIC33E
			MUL.US 	w6,[--W11],w2			; y0*x1
			MUL.SS	w7,[W11--],w4			; y1*x1
			MUL.SU	w7,[W11],w0				; y1*x0

			LAC		w3,A
			ADD		w1,A					
			SFTAC	A,#16					; A=(y1*x0 + y0*x1)>>16
		
			ADD 	w4,[w12],[w12++]
			ADDC	w5,[w12],[w12--]		; A=Rsub*Wr=y1*x1 + (y1*x0 + y0*x1)>>16	
.else

            MPY W5*W6, A; Word1 (signed) x Word2 (unsigned)
			MAC W4*W7, A; Word0 (unsigned) x Word3 (signed)
			SFTAC A, #16; Shift right by 15 bits to align for Q31 format
			MAC W5*W7, A; Word1 (signed) x Word 3 (signed)

.endif
			MOV		[w13++],[W11++]
			MOV		[w13--],[W11++]			; Gr(N-k)=(Radd - (Iadd*Wr-Rsub*Wi))/2 	

									
; Gi(k)=Isub - (Wr*Rsub + Wi*Iadd)
; Gi(N-k)=-Isub - (Wr*Rsub + Wi*Iadd)
			LAC		[++W14],B					
			MOV		[--w14],[w13]			; B=Wi*Iadd
		
			ADD		A						; A=Wi*Iadd+Rsub*Wr
			NEG		A						; A=-(Wi*Iadd+Rsub*Wr)
			LAC		[++W10],B				; B=Isub	
			MOV		[--W10],[w13]
			SFTAC	B,#1					; B=Isub/2 in Q31 format
			ADD		A						; A=(Isub - (Wr*Rsub + Wi*Iadd))/2 in Q31 format
			
			SFTAC	B,#-1
			NEG		B
			ADD		B						; B=(-Isub - (Wr*Rsub + Wi*Iadd))/2 in Q31 format
		
			MOV		[w12++],[W10++]
			MOV		[w12--],[W10++]			; Gi(k)=(Isub - (Wr*Rsub + Wi*Iadd))/2
		
			MOV		[w13++],[W11++]
			MOV		[w13--],[W11++]			; Gi(N-k)=(-Isub - (Wr*Rsub + Wi*Iadd))/2 	
			SUB		W11,#16

		.endm

.ifdef __dsPIC33E
			.macro MERGE_BFLY_MACRO_PSV

; Iadd*Wi 32-bit multiplication
			POP		DSRPAG
			MOV.D	[W9],W6				; (w6,w7)=Wi=(y0,y1)
											; (W11,W11+1)=Iadd=(x0,x1)
			PUSH	DSRPAG
			MOVPAG  #0x01,DSRPAG

            MOV [W11++],w4  ; w4=x0

			MPY W4*W7, B, [W11]-=6,W5
            MAC W5*W6, B, [W11]+=2,W4; Word0 (unsigned) x Word3 (signed)
			SFTAC B, #16; Shift right by 15 bits to align for Q31 format
			MAC W5*W7, B, [W11]+=2,W5; Word1 (signed) x Word 3 (signed)
			
			MOV [w13++],[w14++]
			MOV [w13--],[w14--]

; Rsub*Wi 32-bit multiplication
											; (W11-2,W11-1)=Rsub=(x0,x1)
											; (w6,w7)=Wi=(x0,x1)								
            MPY W5*W6, B; Word1 (signed) x Word2 (unsigned)
			MAC W4*W7, B, [W11]+=2,W4; Word0 (unsigned) x Word3 (signed)
			SFTAC B, #16; Shift right by 15 bits to align for Q31 format
			MAC W5*W7, B, [W11]-=6,W5; Word1 (signed) x Word 3 (signed)
			
; Iadd*Wr 32-bit multiplication		
			POP		DSRPAG									
			MOV.D	[W8],W6				; (w6,w7)=Wr=(y0,y1)
											; (W11+1,W11+2)=Rsub=(x0,x1)	
			PUSH	DSRPAG
			MOVPAG  #0x01,DSRPAG
							
            MPY W5*W6, A; Word1 (signed) x Word2 (unsigned)
			MAC W4*W7, A, [W11]+=2,W4; Word0 (unsigned) x Word3 (signed)
			SFTAC A, #16; Shift right by 15 bits to align for Q31 format
			MAC W5*W7, A, [W11]-=2,W5; Word1 (signed) x Word 3 (signed)
			
; Gr(k)=Radd + (Wr*Iadd - Wi*Rsub)  
; Gr(N-k)=Radd - (Wr*Iadd - Wi*Rsub)  	
			SUB		A						; A=Iadd*Wr-Rsub*Wi		
			LAC		[++W10],B				; B=Radd	
			MOV		[--W10],[w13]
			SFTAC	B,#1					; B=Radd/2 in Q31 format
			ADD		A						; A=(Radd + (Iadd*Wr-Rsub*Wi))/2 in Q31 format
			
			SFTAC	B,#-1
			SUB		B						; B=(Radd - (Iadd*Wr-Rsub*Wi))/2 in Q31 format
		
			MOV		[w12++],[W10++]
			MOV		[w12--],[W10++]			; Gr(k)=(Radd + (Iadd*Wr-Rsub*Wi))/2  
	
	
; Rsub*Wr 32-bit multiplication
											; (w6,w7)=Wr=(y0,y1)
											; (W11-2,W11-1)=Rsub=(x0,x1)
            MPY W5*W6, A; Word1 (signed) x Word2 (unsigned)
			MAC W4*W7, A; Word0 (unsigned) x Word3 (signed)
			SFTAC A, #16; Shift right by 15 bits to align for Q31 format
			MAC W5*W7, A; Word1 (signed) x Word 3 (signed)

			MOV		[w13++],[W11++]
			MOV		[w13--],[W11++]			; Gr(N-k)=(Radd - (Iadd*Wr-Rsub*Wi))/2 	

									
; Gi(k)=Isub - (Wr*Rsub + Wi*Iadd)
; Gi(N-k)=-Isub - (Wr*Rsub + Wi*Iadd)
			LAC		[++W14],B					
			MOV		[--w14],[w13]			; B=Wi*Iadd
		
			ADD		A						; A=Wi*Iadd+Rsub*Wr
			NEG		A						; A=-(Wi*Iadd+Rsub*Wr)
			LAC		[++W10],B				; B=Isub	
			MOV		[--W10],[w13]
			SFTAC	B,#1					; B=Isub/2 in Q31 format
			ADD		A						; A=(Isub - (Wr*Rsub + Wi*Iadd))/2 in Q31 format
			
			SFTAC	B,#-1
			NEG		B
			ADD		B						; B=(-Isub - (Wr*Rsub + Wi*Iadd))/2 in Q31 format
		
			MOV		[w12++],[W10++]
			MOV		[w12--],[W10++]			; Gi(k)=(Isub - (Wr*Rsub + Wi*Iadd))/2
		
			MOV		[w13++],[W11++]
			MOV		[w13--],[W11++]			; Gi(N-k)=(-Isub - (Wr*Rsub + Wi*Iadd))/2 	
			SUB		W11,#16

		.endm
.endif

;............................................................................


; Local Stack Frame
;............................................................................
;   |_______|
;   |_______|<- Stack Pointer                          	SP 
;   |_______|<- Stack Pointer Return                   	SP+6
;   |_______|<- Twiddle Factor INDEX	       			FP+4
;   |_______|<- Temp                               		FP                                              
;............................................................................

    .equ    SP_RET,     6
	.equ	TF_INDX,  	4
	.equ	TEMP,		0
 

;............................................................................
; Real FFT Calculation
;............................................................................
_realFft32bIP:
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
			LNK     #8
;............................................................................
;;	PSVPAG = __builtin_psvpage(&twiddleFactor);
;;	CORCONbits.PSV = 1;	
.ifdef __dsPIC33E
			BSET	CORCON,#0           ; Set IF bit
			BSET	CORCON,#13          ; Set mixed-sign mode
			BCLR	CORCON,#12

        	MOV	    #COEFFS_IN_DATA,W7	; W7 = COEFFS_IN_DATA
	        CP	    W7,W3				; W7 - W3
	        BRA	    Z,_NOPSV            ; IF W3 = COEFFS_IN_DATA
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
			MOV		w2,W9					; W9 ---> Wi = SIN(*)
	
			MOV		#TF_FFT_SIZE,W10
			ADD		w2,W10,w2
			MOV		w2,W8					; W8 ---> Wr = COS(*)


			MOV		w1,W10					; W10  ---> Pr[0], first bin
			MOV		#8,w2
			SL		w2,w0,w2
			ADD		w1,w2,W11				; W11 ---> Pr[N], last bin

			LSR		w2,#4,w6				; N/2
			SUB		w6,#2,w6				; w6 = BIN_CNTR = (N/2)-2


			SUBR	w0,#TF_FFT_STG,w1
			MOV		#2,w2					; ************CHECK THIS			
			SL		w2,w1,w2				; TF_INDX=TF_FFT_SIZE/FFT_SIZE
			MOV		w2,[w14+TF_INDX]


			ADD		W8,w2,W8				; Next Twiddle Factor (Wr)
			ADD		W9,w2,W9				; Next Twiddle Factor (Wi)

			MOV		#ACCAL,w12				; w12 ---> A
			MOV		#ACCBL,w13				; W13 ---> B
			


; DC and Nyquist Bin
			ADD		W10,#8,w7

			LAC		[--w7],A				; A=Pi[0]	
			MOV		[--w7],[w12]	

			LAC		[--w7],B				; B=Pr[0]	
			MOV		[--w7],[w13]
			ADD		A						; A=(Pr[0]+Pi[0])/2
			SFTAC	A,#1	
			SUB		B						; Gr[N]=(Pr[0]-Pi[0])/2

			MOV		[w12++],[W10++]
			MOV		[w12--],[W10++]			; Gr[0]=(Pr[0]+Pi[0])/2
			MOV		#0,w0
			MOV		w0,[W10++]
			MOV		w0,[W10++]				; Gi[0]=0	

			MOV		[w13++],[W11++]
			MOV		[w13--],[W11++]			; Gr[N]=(Pr[0]-Pi[0])/2
 			MOV		w0,[W11++]
			MOV		w0,[W11++]				; Gi[N]=0

			SUB		W11,#16					; W11---> Gr[N-1], W10---> Gr[1]


; Bin 1 to N-1, k=1:(N/2-1)
			DO 		w6,BIN_END_PSV
BIN_START_PSV:
			LAC		[++W11],A				; A=Pr[N-k]	
			MOV		[--W11],[w12]	
			LAC		[++W10],B				; B=Pr[k]	
			MOV		[--W10],[w13]
			ADD		A						; A=(Pr[k]+Pr[N-k])/2
			SFTAC	A,#1	
			SUB		B						; Gr[N]=(Pr[k]-Pr[N-k])/2

			MOV		[w12++],[W10++]
			MOV		[w12--],[W10++]			; Radd[k]=(Pr[k]+Pr[N-k])/2


			MOV		[w13++],[W11++]
			MOV		[w13--],[W11++]			; Rsub[k]=(Pr[k]-Pr[N-k])/2
	

			LAC		[++W11],A				; A=Pi[N-k]	
			MOV		[--W11],[w12]	
			LAC		[++W10],B				; B=Pi[k]	
			MOV		[--W10],[w13]
			ADD		A						; A=(Pi[k]+Pi[N-k])/2
			SFTAC	A,#1	
			SUB		B						; Gr[N]=(Pi[k]-Pi[N-k])/2

			MOV		[w12++],[W11++]
			MOV		[w12--],[W11--]			; Iadd[k]=(Pi[k]+Pi[N-k])/2


			MOV		[w13++],[W10++]
			MOV		[w13--],[W10++]			; Isub[k]=(Pi[k]-Pi[N-k])/2

		
											; W11---> Gi[N-k],
			SUB		W10,#8					; W10---> Gr[k]

			MERGE_BFLY_MACRO_PSV
		
			MOV		[w14+TF_INDX],w0
			ADD		W8,w0,W8				; Next Twiddle Factor (Wr)
BIN_END_PSV:	ADD		W9,w0,W9				; Next Twiddle Factor (Wi)


			LAC		[++W10],A				; B=Pr(N/2)	
			MOV		[--W10],[w12]
			SFTAC	A,#1					
			MOV		[w12++],[W10++]
			MOV		[w12--],[W10++]			; Gr(N/2)=Pr(N/2)/2

			LAC		[++W10],A				; B=Pi(N/2)	
			MOV		[--W10],[w12]
			SFTAC	A,#1					
			MOV		[w12++],[W10++]
			MOV		[w12--],[W10++]			; Gi(N/2)=Pi(N/2)/2

			POP     DSRPAG
		    BRA     _DONEREALFFT

_COPYTOSTACK:
            ; save SP_RET
			MOV		W15,[W14+SP_RET]
			MOV     W15,W7
			
        	MOV	   #TF_FFT_SIZE,W11		; 
	        SL     W11,W12               ; 
         	ADD    W11,W12,W11            ; TABLE_SIZE IN WORDS
			
        	; COPY ENTIRE TWIDDLE TABLE TO STACK
	        DEC    W11,W13           ; REPEAT COUNT
	        REPEAT W13              ; SET REPEAT COUNT
	        MOV    [W2++],[w15++]   ; copy from PSV to stack
	
; Store Input Parameters
			MOV		w7,W9					; W9 ---> Wi = SIN(*)
	
			MOV		#TF_FFT_SIZE,W10
			ADD		w7,W10,w7
			MOV		w7,W8					; W8 ---> Wr = COS(*)

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
			MOV		w2,W9					; W9 ---> Wi = SIN(*)
	
			MOV		#TF_FFT_SIZE,W10
			ADD		w2,W10,w2
			MOV		w2,W8					; W8 ---> Wr = COS(*)

_INPUTPARAMSSTORED:
			MOV		w1,W10					; W10  ---> Pr[0], first bin
			MOV		#8,w2
			SL		w2,w0,w2
			ADD		w1,w2,W11				; W11 ---> Pr[N], last bin

			LSR		w2,#4,w6				; N/2
			SUB		w6,#2,w6				; w6 = BIN_CNTR = (N/2)-2


			SUBR	w0,#TF_FFT_STG,w1
			MOV		#2,w2					; ************CHECK THIS			
			SL		w2,w1,w2				; TF_INDX=TF_FFT_SIZE/FFT_SIZE
			MOV		w2,[w14+TF_INDX]


			ADD		W8,w2,W8				; Next Twiddle Factor (Wr)
			ADD		W9,w2,W9				; Next Twiddle Factor (Wi)

			MOV		#ACCAL,w12				; w12 ---> A
			MOV		#ACCBL,w13				; W13 ---> B
			


; DC and Nyquist Bin
			ADD		W10,#8,w7

			LAC		[--w7],A				; A=Pi[0]	
			MOV		[--w7],[w12]	

			LAC		[--w7],B				; B=Pr[0]	
			MOV		[--w7],[w13]
			ADD		A						; A=(Pr[0]+Pi[0])/2
			SFTAC	A,#1	
			SUB		B						; Gr[N]=(Pr[0]-Pi[0])/2

			MOV		[w12++],[W10++]
			MOV		[w12--],[W10++]			; Gr[0]=(Pr[0]+Pi[0])/2
			MOV		#0,w0
			MOV		w0,[W10++]
			MOV		w0,[W10++]				; Gi[0]=0	

			MOV		[w13++],[W11++]
			MOV		[w13--],[W11++]			; Gr[N]=(Pr[0]-Pi[0])/2
 			MOV		w0,[W11++]
			MOV		w0,[W11++]				; Gi[N]=0

			SUB		W11,#16					; W11---> Gr[N-1], W10---> Gr[1]


; Bin 1 to N-1, k=1:(N/2-1)
			DO 		w6,BIN_END
BIN_START:
			LAC		[++W11],A				; A=Pr[N-k]	
			MOV		[--W11],[w12]	
			LAC		[++W10],B				; B=Pr[k]	
			MOV		[--W10],[w13]
			ADD		A						; A=(Pr[k]+Pr[N-k])/2
			SFTAC	A,#1	
			SUB		B						; Gr[N]=(Pr[k]-Pr[N-k])/2

			MOV		[w12++],[W10++]
			MOV		[w12--],[W10++]			; Radd[k]=(Pr[k]+Pr[N-k])/2


			MOV		[w13++],[W11++]
			MOV		[w13--],[W11++]			; Rsub[k]=(Pr[k]-Pr[N-k])/2
	

			LAC		[++W11],A				; A=Pi[N-k]	
			MOV		[--W11],[w12]	
			LAC		[++W10],B				; B=Pi[k]	
			MOV		[--W10],[w13]
			ADD		A						; A=(Pi[k]+Pi[N-k])/2
			SFTAC	A,#1	
			SUB		B						; Gr[N]=(Pi[k]-Pi[N-k])/2

			MOV		[w12++],[W11++]
			MOV		[w12--],[W11--]			; Iadd[k]=(Pi[k]+Pi[N-k])/2


			MOV		[w13++],[W10++]
			MOV		[w13--],[W10++]			; Isub[k]=(Pi[k]-Pi[N-k])/2

		
											; W11---> Gi[N-k],
			SUB		W10,#8					; W10---> Gr[k]

			MERGE_BFLY_MACRO
		
			MOV		[w14+TF_INDX],w0
			ADD		W8,w0,W8				; Next Twiddle Factor (Wr)
BIN_END:	ADD		W9,w0,W9				; Next Twiddle Factor (Wi)


			LAC		[++W10],A				; B=Pr(N/2)	
			MOV		[--W10],[w12]
			SFTAC	A,#1					
			MOV		[w12++],[W10++]
			MOV		[w12--],[W10++]			; Gr(N/2)=Pr(N/2)/2

			LAC		[++W10],A				; B=Pi(N/2)	
			MOV		[--W10],[w12]
			SFTAC	A,#1					
			MOV		[w12++],[W10++]
			MOV		[w12--],[W10++]			; Gi(N/2)=Pi(N/2)/2

			MOV		[W14+SP_RET],W15
			
_DONEREALFFT:
;............................................................................
; Context Restore
			ULNK
	.ifdef __dsPIC33E
			POP 	DSRPAG
	.else
			POP		PSVPAG
	.endif			
			POP		CORCON
			POP.D	w12					; {w12,w13} from TOS
			POP.D	W8					; {W8,W9} from TOS
			POP.D	W10					; {W10,W11} from TOS
;............................................................................
			RETURN	

