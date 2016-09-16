/*******************************************************************************
 * Copyright (c) 2014-2015 IBM Corporation.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    IBM Zurich Research Lab - initial API, implementation and documentation
 *******************************************************************************/

//! \file
#ifndef _oslmic_h_
#define _oslmic_h_

// Dependencies required for the LoRa MAC in C to run.
// These settings can be adapted to the underlying system.
// You should not, however, change the lmic.[hc]

#include "lmic/lmic-types.h"
#include <string.h>
#include <stdio.h>
#include "hal.h"
#include "rtimer.h"

#ifndef OSLMIC_FROM_CONTIKI

#ifdef ASSERT
#undef ASSERT
#endif

#define EV(a,b,c) /**/
#define DO_DEVDB(field1,field2) /**/
#if !defined(CFG_noassert)
#define ASSERT(cond) do { \
	if(!(cond)) { \
		printf("LoRa: Assertion failed in file %s (line: %d)\n", __FILE__, __LINE__); \
		hal_failed(); \
	} \
} while(0)
#else
#define ASSERT(cond) /**/
#endif

#endif

#define os_clearMem(a,b)   memset(a,0,b)
#define os_copyMem(a,b,c)  memcpy(a,b,c)

typedef     struct osjob_t osjob_t;
typedef      struct band_t band_t;
typedef   struct chnldef_t chnldef_t;
typedef   struct rxsched_t rxsched_t;
typedef   struct bcninfo_t bcninfo_t;
typedef        const u1_t* xref2cu1_t;
typedef              u1_t* xref2u1_t;
#define TYPEDEF_xref2rps_t     typedef         rps_t* xref2rps_t
#define TYPEDEF_xref2rxsched_t typedef     rxsched_t* xref2rxsched_t
#define TYPEDEF_xref2chnldef_t typedef     chnldef_t* xref2chnldef_t
#define TYPEDEF_xref2band_t    typedef        band_t* xref2band_t
#define TYPEDEF_xref2osjob_t   typedef       osjob_t* xref2osjob_t

#define SIZEOFEXPR(x) sizeof(x)

#define ON_LMIC_EVENT(ev)  onEvent(ev)
#define DECL_ON_LMIC_EVENT void onEvent(ev_t e)

extern u4_t AESAUX[];
extern u4_t AESKEY[];
#define AESkey ((u1_t*)AESKEY)
#define AESaux ((u1_t*)AESAUX)
#define FUNC_ADDR(func) (&(func))
#define DEFINE_LMIC  struct lmic_t LMIC
#define DECLARE_LMIC extern struct lmic_t LMIC


//================================================================================


#ifndef RX_RAMPUP
#define RX_RAMPUP  (us2osticks(2000))
#endif
#ifndef TX_RAMPUP
#define TX_RAMPUP  (us2osticks(2000))
#endif

#ifndef OSTICKS_PER_SEC_DIVIDER
#define OSTICKS_PER_SEC_DIVIDER (1)
#endif

#ifndef OSTICKS_PER_SEC
#define OSTICKS_PER_SEC (RTIMER_ARCH_SECOND / OSTICKS_PER_SEC_DIVIDER)
#endif

/*
 * Not sure why this is required, but apparently it is.
 */
#if OSTICKS_PER_SEC < 10000 || OSTICKS_PER_SEC > 64516
#error Illegal OSTICKS_PER_SEC - must be in range [10000:64516]. One tick must be 15.5us .. 100us long.
#endif

#if !HAS_ostick_conv
#define us2osticks(us)   ((ostime_t)( ((s8_t)(us) * OSTICKS_PER_SEC) / 1000000))
#define ms2osticks(ms)   ((ostime_t)( ((s8_t)(ms) * OSTICKS_PER_SEC)    / 1000))
#define sec2osticks(sec) ((ostime_t)( (s8_t)(sec) * OSTICKS_PER_SEC))
#define osticks2ms(os)   ((s4_t)(((os)*(s8_t)1000    ) / OSTICKS_PER_SEC))
#define osticks2us(os)   ((s4_t)(((os)*(s8_t)1000000 ) / OSTICKS_PER_SEC))
// Special versions
#define us2osticksCeil(us)  ((ostime_t)( ((s8_t)(us) * OSTICKS_PER_SEC + 999999) / 1000000))
#define us2osticksRound(us) ((ostime_t)( ((s8_t)(us) * OSTICKS_PER_SEC + 500000) / 1000000))
#define ms2osticksCeil(ms)  ((ostime_t)( ((s8_t)(ms) * OSTICKS_PER_SEC + 999) / 1000))
#define ms2osticksRound(ms) ((ostime_t)( ((s8_t)(ms) * OSTICKS_PER_SEC + 500) / 1000))
#endif

struct osjob_t;  // fwd decl.
typedef void (*osjobcb_t) (struct osjob_t*);
struct osjob_t {
	struct rtimer timer;
	rtimer_clock_t rtimer_deadline;
	osjobcb_t func;
	bool run_immediately;
	bool pending;
};
TYPEDEF_xref2osjob_t;

// ======================================================================
// AES support
// !!Keep in sync with lorabase.hpp!!

#ifndef AES_ENC  // if AES_ENC is defined as macro all other values must be too
#define AES_ENC       0x00
#define AES_DEC       0x01
#define AES_MIC       0x02
#define AES_CTR       0x04
#define AES_MICNOAUX  0x08
#endif
#ifndef AESkey  // if AESkey is defined as macro all other values must be too
extern xref2u1_t AESkey;
extern xref2u1_t AESaux;
#endif

#ifndef os_aes
u4_t os_aes (u1_t mode, xref2u1_t buf, u2_t len);
#endif

u1_t radio_rand1 (void);
u2_t os_getRndU2();
#define os_getRndU1() radio_rand1()
void radio_irq_handler (u1_t dio);
void os_setCallback (osjob_t* job, osjobcb_t cb);
void os_setTimedCallback (osjob_t* job, ostime_t time, osjobcb_t cb);
void os_clearCallback (osjob_t* job);


#endif // _oslmic_h_
