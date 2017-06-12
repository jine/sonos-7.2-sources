@ "This software program is available to you under a choice of one of two 
@ licenses.  You may choose to be licensed under either the GNU General Public 
@ License (GPL) Version 2, June 1991, available at 
@ http://www.fsf.org/copyleft/gpl.html, or the BSD License, the text of
@ which follows:
@ 
@ Copyright (c) 1996-2005, Intel Corporation. All rights reserved.
@ 
@ Redistribution and use in source and binary forms, with or without 
@ modification, are permitted provided that the following conditions are met:
@ 
@ Redistributions of source code must retain the above copyright notice, this 
@ list of conditions and the following disclaimer. 
@ 
@ Redistributions in binary form must reproduce the above copyright notice, this
@ list of conditions and the following disclaimer in the documentation and/or 
@ other materials provided with the distribution. 
@ 
@ Neither the name of the Intel Corporation ("Intel") nor the names of its 
@ contributors may be used to endorse or promote products derived from this
@ software without specific prior written permission. 
@ 
@ THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
@ AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
@ IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
@ DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
@ FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
@ DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
@ SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
@ CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
@ OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
@ OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
@
@  FILENAME:       clkmgr_gnu.S
@
@  PURPOSE:        Provides low level PMU primitive functions written specifically for
@                  the Monanhans/Zylonite processor/platform.  Specially design to fit  
@                  into Intel VTUNE Architecture
@                  
@ 
@  LAST MODIFIED:  3/1/05
@******************************************************************************
@
@ List of primitive functions in this source code include:
@

   .global pxa_clkcfg_read          
   .global pxa_clkcfg_write

    .text

@
@ pxa_clkcfg_read - Read the PMU Register
@
@ Description:
@ 	This routine reads the designated PMU register via CoProcesser 14.
@
@ Input Parameters:
@
@ Returns:
@	r0 - clkcfg value
@    
@ Registers Modified:
@	CoProcessor Register Modified: None
@	General Purpose Registers Modified: None	
@
@

pxa_clkcfg_read:
	mrc  p14, 0, r0, c6, c0, 0		@ Read clkcfg
    	bx lr          @ return
	 
  

@
@ pxa_clkcfg_write - Writes to the PMU Register
@
@ Description:
@ 	This routine writes to the designated ClkCFG register via CoProcesser 14.
@
@ Input Parameters:   
@       r0 - arg1 - Value to write to ClkCFG register
@

@ Returns:
@	None
@
@ Registers Modified:
@	CoProcessor Register Modified: ClkCFG Register
@	General Purpose Registers Modified: None
@
@ NOTE   
@	Error checking not included
@

pxa_clkcfg_write:
	mcr	 p14, 0, r0, c6, c0, 0	@ Write ClkCFG
	bx	 lr         @ return
    
