/*
 * $Id$
 * Portable Audio I/O Library for ASIO Drivers
 *
 * Author: Stephane Letz
 * Based on the Open Source API proposed by Ross Bencina
 * Copyright (c) 2000-2002 Stephane Letz, Phil Burk, Ross Bencina
 * Blocking i/o implementation by Sven Fischer, Institute of Hearing
 * Technology and Audiology (www.hoertechnik-audiologie.de)
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*
 * The text above constitutes the entire PortAudio license; however,
 * the PortAudio community also makes the following non-binding requests:
 *
 * Any person wishing to distribute modifications to the Software is
 * requested to send the modifications to the original developer so that
 * they can be incorporated into the canonical version. It is also
 * requested that these non-binding requests be included along with the
 * license above.
 */

/* Modification History

        08-03-01 First version : Stephane Letz
        08-06-01 Tweaks for PC, use C++, buffer allocation, Float32 to Int32 conversion : Phil Burk
        08-20-01 More conversion, PA_StreamTime, Pa_GetHostError : Stephane Letz
        08-21-01 PaUInt8 bug correction, implementation of ASIOSTFloat32LSB and ASIOSTFloat32MSB native formats : Stephane Letz
        08-24-01 MAX_INT32_FP hack, another Uint8 fix : Stephane and Phil
        08-27-01 Implementation of hostBufferSize < userBufferSize case, better management of the output buffer when
                 the stream is stopped : Stephane Letz
        08-28-01 Check the stream pointer for null in bufferSwitchTimeInfo, correct bug in bufferSwitchTimeInfo when
                 the stream is stopped : Stephane Letz
        10-12-01 Correct the PaHost_CalcNumHostBuffers function: computes FramesPerHostBuffer to be the lowest that
                 respect requested FramesPerUserBuffer and userBuffersPerHostBuffer : Stephane Letz
        10-26-01 Management of hostBufferSize and userBufferSize of any size : Stephane Letz
        10-27-01 Improve calculus of hostBufferSize to be multiple or divisor of userBufferSize if possible : Stephane and Phil
        10-29-01 Change MAX_INT32_FP to (2147483520.0f) to prevent roundup to 0x80000000 : Phil Burk
        10-31-01 Clear the output buffer and user buffers in PaHost_StartOutput, correct bug in GetFirstMultiple : Stephane Letz
        11-06-01 Rename functions : Stephane Letz
        11-08-01 New Pa_ASIO_Adaptor_Init function to init Callback adpatation variables, cleanup of Pa_ASIO_Callback_Input: Stephane Letz
        11-29-01 Break apart device loading to debug random failure in Pa_ASIO_QueryDeviceInfo ; Phil Burk
        01-03-02 Deallocate all resources in PaHost_Term for cases where Pa_CloseStream is not called properly :  Stephane Letz
        02-01-02 Cleanup, test of multiple-stream opening : Stephane Letz
        19-02-02 New Pa_ASIO_loadDriver that calls CoInitialize on each thread on Windows : Stephane Letz
        09-04-02 Correct error code management in PaHost_Term, removes various compiler warning : Stephane Letz
        12-04-02 Add Mac includes for <Devices.h> and <Timer.h> : Phil Burk
        13-04-02 Removes another compiler warning : Stephane Letz
        30-04-02 Pa_ASIO_QueryDeviceInfo bug correction, memory allocation checking, better error handling : D Viens, P Burk, S Letz
        12-06-02 Rehashed into new multi-api infrastructure, added support for all ASIO sample formats : Ross Bencina
        18-06-02 Added pa_asio.h, PaAsio_GetAvailableLatencyValues() : Ross B.
        21-06-02 Added SelectHostBufferSize() which selects host buffer size based on user latency parameters : Ross Bencina
        ** NOTE  maintenance history is now stored in CVS **
*/

/** @file
    @ingroup hostapi_src

    Note that specific support for paInputUnderflow, paOutputOverflow and
    paNeverDropInput is not necessary or possible with this driver due to the
    synchronous full duplex double-buffered architecture of ASIO.
*/


#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#if _WINDOWS
    #include <windows.h>
    #include <mmsystem.h>
#else
    #include <time.h>
    #include <unistd.h>
    typedef u_int32_t DWORD;
#endif

#include "pevents.h"

#include "portaudio.h"
#include "pa_cwasio.h"
#include "pa_util.h"
#include "pa_allocation.h"
#include "pa_hostapi.h"
#include "pa_stream.h"
#include "pa_cpuload.h"
#include "pa_process.h"
#include "pa_debugprint.h"
#include "pa_ringbuffer.h"

#if _WINDOWS
    #include "pa_win_coinitialize.h"
    #include "pa_win_util.h"
#endif

 #include <cwASIO.h>

/* winmm.lib is needed for timeGetTime() (this is in winmm.a if you're using gcc) */
#if defined(WIN32)
    #define WINDOWS 1
    #if (defined(_MSC_VER) && (_MSC_VER >= 1200)) /* MSC version 6 and above */
        #pragma comment(lib, "winmm.lib")
    #endif
#endif


/* prototypes for functions declared in this file */

PaError PaCwAsio_Initialize( PaUtilHostApiRepresentation **hostApi, PaHostApiIndex hostApiIndex );
static void Terminate( struct PaUtilHostApiRepresentation *hostApi );
static PaError OpenStream( struct PaUtilHostApiRepresentation *hostApi,
                           PaStream** s,
                           const PaStreamParameters *inputParameters,
                           const PaStreamParameters *outputParameters,
                           double sampleRate,
                           unsigned long framesPerBuffer,
                           PaStreamFlags streamFlags,
                           PaStreamCallback *streamCallback,
                           void *userData );
static PaError IsFormatSupported( struct PaUtilHostApiRepresentation *hostApi,
                                  const PaStreamParameters *inputParameters,
                                  const PaStreamParameters *outputParameters,
                                  double sampleRate );
static PaError CloseStream( PaStream* stream );
static PaError StartStream( PaStream *stream );
static PaError StopStream( PaStream *stream );
static PaError AbortStream( PaStream *stream );
static PaError IsStreamStopped( PaStream *s );
static PaError IsStreamActive( PaStream *stream );
static PaTime GetStreamTime( PaStream *stream );
static double GetStreamCpuLoad( PaStream* stream );
static PaError ReadStream( PaStream* stream, void *buffer, unsigned long frames );
static PaError WriteStream( PaStream* stream, const void *buffer, unsigned long frames );
static signed long GetStreamReadAvailable( PaStream* stream );
static signed long GetStreamWriteAvailable( PaStream* stream );

/* Blocking i/o callback function. */
static int BlockingIoPaCallback(const void                     *inputBuffer    ,
                                      void                     *outputBuffer   ,
                                      unsigned long             framesPerBuffer,
                                const PaStreamCallbackTimeInfo *timeInfo       ,
                                      PaStreamCallbackFlags     statusFlags    ,
                                      void                     *userData       );

/* our ASIO callback functions */

static void bufferSwitch(long index, cwASIOBool processNow);
static struct cwASIOTime *bufferSwitchTimeInfo(struct cwASIOTime *timeInfo, long index, cwASIOBool processNow);
static void sampleRateChanged(cwASIOSampleRate sRate);
static long asioMessages(long selector, long value, void* message, double* opt);

static struct cwASIOCallbacks asioCallbacks_ =
    { bufferSwitch, sampleRateChanged, asioMessages, bufferSwitchTimeInfo };


#define PA_CWASIO_SET_LAST_HOST_ERROR( errorCode, errorText ) \
    PaUtil_SetLastHostErrorInfo( paASIO, errorCode, errorText )


static void PaCwAsio_SetLastSystemError( DWORD errorCode )
{
#if _WINDOWS
    PaWinUtil_SetLastSystemErrorInfo( paCwASIO, errorCode );
#else
    PaUtil_SetLastHostErrorInfo( paASIO, errorCode, "system error" );
#endif
}

static DWORD getLastError()
{
#if _WINDOWS
    return GetLastError();
#else
    DWORD result = DWORD(errno);
    return result;
#endif
}

#define PA_CWASIO_SET_LAST_SYSTEM_ERROR( errorCode ) \
    PaCwAsio_SetLastSystemError( errorCode )


static const char* PaCwAsio_GetAsioErrorText( cwASIOError asioError )
{
    const char *result;

    switch( asioError ){
        case ASE_OK:
        case ASE_SUCCESS:           result = "Success"; break;
        case ASE_NotPresent:        result = "Hardware input or output is not present or available"; break;
        case ASE_HWMalfunction:     result = "Hardware is malfunctioning"; break;
        case ASE_InvalidParameter:  result = "Input parameter invalid"; break;
        case ASE_InvalidMode:       result = "Hardware is in a bad mode or used in a bad mode"; break;
        case ASE_SPNotAdvancing:    result = "Hardware is not running when sample position is inquired"; break;
        case ASE_NoClock:           result = "Sample clock or rate cannot be determined or is not present"; break;
        case ASE_NoMemory:          result = "Not enough memory for completing the request"; break;
        default:                    result = "Unknown ASIO error"; break;
    }

    return result;
}


#define PA_CWASIO_SET_LAST_ASIO_ERROR( asioError ) \
    PaUtil_SetLastHostErrorInfo( paCwASIO, asioError, PaCwAsio_GetAsioErrorText( asioError ) )




// Atomic increment and decrement operations
#if MAC
    /* need to be implemented on Mac */
    inline long PaCwAsio_AtomicIncrement(volatile long* v) {return ++(*(long*)(v));}
    inline long PaCwAsio_AtomicDecrement(volatile long* v) {return --(*(long*)(v));}
#elif WINDOWS
    inline long PaCwAsio_AtomicIncrement(volatile long* v) {return InterlockedIncrement((long*)(v));}
    inline long PaCwAsio_AtomicDecrement(volatile long* v) {return InterlockedDecrement((long*)(v));}
#else
    inline long PaCwAsio_AtomicIncrement(volatile long* v) {return __sync_add_and_fetch((long*)(v), 1L);}
    inline long PaCwAsio_AtomicDecrement(volatile long* v) {return __sync_sub_and_fetch((long*)(v), 1L);}
#endif



// Sleep function that sleeps for milliseconds
#if WINDOWS
    void PaCwAsio_SleepMilliseconds(unsigned milliseconds) {Sleep(milliseconds);}
#else
    void PaCwAsio_SleepMilliseconds(unsigned milliseconds) {useconds_t usec = milliseconds * 1000U; usleep(usec);}
#endif



typedef struct CwAsioDriverInfo
{
    struct cwASIODriverInfo base;
    char clsid[64];
    char desc[128];
}
CwAsioDriverInfo;

typedef struct PaCwAsioDriverInfo
{
    CwAsioDriverInfo cwAsioDriverInfo;
    long inputChannelCount, outputChannelCount;
    long bufferMinSize, bufferMaxSize, bufferPreferredSize, bufferGranularity;
    bool postOutput;
}
PaCwAsioDriverInfo;

typedef struct CwAsioDriverInfos
{
    CwAsioDriverInfo asioDriverInfos[32];
    size_t size;
}
CwAsioDriverInfos;


/* PaCwAsioHostApiRepresentation - host api datastructure specific to this implementation */

typedef struct
{
    PaUtilHostApiRepresentation inheritedHostApiRep;
    PaUtilStreamInterface callbackStreamInterface;
    PaUtilStreamInterface blockingStreamInterface;

    PaUtilAllocationGroup *allocations;

    CwAsioDriverInfos driverInfos;
    
    void *systemSpecific;

    /* the ASIO C API only allows one ASIO driver to be open at a time,
        so we keep track of whether we have the driver open here, and
        use this information to return errors from OpenStream if the
        driver is already open.

        openAsioDeviceIndex will be PaNoDevice if there is no device open
        and a valid pa_asio (not global) device index otherwise.

        openAsioDriverInfo is populated with the driver info for the
        currently open device (if any)
    */
    PaDeviceIndex openAsioDeviceIndex;
    PaCwAsioDriverInfo openAsioDriverInfo;
}
PaCwAsioHostApiRepresentation;

/* cwASIO convenience functions emulating the old ASIO SDK functions */
static struct cwASIODriver* theAsioDriver = NULL;

static cwASIOError cwASIOLoad(char const* path) {
    if (theAsioDriver)
        return ASE_NoMemory;
    return cwASIOload(path, &theAsioDriver);
}

static cwASIOError cwASIOUnload(void) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_InvalidParameter;
    cwASIOunload(theAsioDriver);
    theAsioDriver = NULL;
    return ASE_OK;
}

static cwASIOError cwASIOExit(void) {
    return ASE_OK;
}

/* because of Steinberg's goof the ASIO driver DLLs under WIndows use the __thiscall calling convention */
/* the following functions using inline assembler will fix this for the differenct platforms and compilers if necessary */
#if INTPTR_MAX != INT64_MAX && defined(_WIN32) && (defined(_MSC_VER) || defined(__BCPLUSPLUS__) || defined(__BORLANDC__))
/* 32 bit Windows platform and MSVC or Borland compiler */
static cwASIOError cwASIOInit(struct cwASIODriverInfo *info) {
    if(!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    {
        void *f = (void*) theAsioDriver->lpVtbl->init;
        void *p =         info ? info->sysRef : 0;
        cwASIOError retval;
        __asm
        {
            mov ecx, theAsioDriver; /* this pointer needs to be in ECX register */
            push p;
            call f;
            mov retval, eax; /* return value is in EAX register */
        }
        if (!retval)
            return ASE_NotPresent;
    }
    if(info) {
        info->asioVersion = 2;
        {
            void *f = (void*) theAsioDriver->lpVtbl->getDriverName;
            char *p =         info->name;
            __asm
            {
                mov ecx, theAsioDriver; /* this pointer needs to be in ECX register */
                push p;
                call f;
            }
        }
        {
            void *f = (void*) theAsioDriver->lpVtbl->getDriverVersion;
            long retval;
            __asm
            {
                mov ecx, theAsioDriver; /* this pointer needs to be in ECX register */
                call f;
                mov retval, eax; /* return value is in EAX register */
            }
            info->driverVersion = retval;
        }
        {
            void *f = (void*) theAsioDriver->lpVtbl->getErrorMessage;
            char *p = (char*) info->errorMessage;
            __asm
            {
                mov ecx, theAsioDriver; /* this pointer needs to be in ECX register */
                push p;
                call f;
            }
        }
    }
    return ASE_OK;
}

static cwASIOError cwASIOStart(void) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->start;
    cwASIOError retval;
    __asm
    {
        mov ecx, theAsioDriver; /* this pointer needs to be in ECX register */
        call f;
        mov retval, eax; /* return value is in EAX register */
    }
    return retval;
}

static cwASIOError cwASIOStop(void) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->stop;
    cwASIOError retval;
    __asm
    {
        mov ecx, theAsioDriver; /* this pointer needs to be in ECX register */
        call f;
        mov retval, eax; /* return value is in EAX register */
    }
    return retval;
}

static cwASIOError cwASIOGetChannels(long *numInputChannels, long *numOutputChannels) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->getChannels;
    cwASIOError retval;
    __asm
    {
        mov ecx, theAsioDriver; /* this pointer needs to be in ECX register */
        push numOutputChannels;
        push numInputChannels;
        call f;
        mov retval, eax; /* return value is in EAX register */
    }
    return retval;
}

static cwASIOError cwASIOGetLatencies(long *inputLatency, long *outputLatency) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->getLatencies;
    cwASIOError retval;
    __asm
    {
        mov ecx, theAsioDriver; /* this pointer needs to be in ECX register */
        push outputLatency;
        push inputLatency;
        call f;
        mov retval, eax; /* return value is in EAX register */
    }
    return retval;
}

static cwASIOError cwASIOGetBufferSize(long *minSize, long *maxSize, long *preferredSize, long *granularity) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->getBufferSize;
    cwASIOError retval;
    __asm
    {
        mov ecx, theAsioDriver; /* this pointer needs to be in ECX register */
        push granularity;
        push preferredSize;
        push maxSize;
        push minSize;
        call f;
        mov retval, eax; /* return value is in EAX register */
    }
    return retval;
}

static cwASIOError cwASIOCanSampleRate(cwASIOSampleRate sampleRate) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*)  theAsioDriver->lpVtbl->canSampleRate;
    void *p = (void*) &sampleRate;
    cwASIOError retval;
    __asm
    {
        mov ecx, theAsioDriver; /* this pointer needs to be in ECX register */
        mov eax, p;
        push [eax+04h];
        push [eax];
        call f;
        mov retval, eax; /* return value is in EAX register */
    }
    return retval;
}

static cwASIOError cwASIOGetSampleRate(cwASIOSampleRate *currentRate) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void* f = (void*)theAsioDriver->lpVtbl->getSampleRate;
    cwASIOError retval;
    __asm
    {
        mov ecx, theAsioDriver; /* this pointer needs to be in ECX register */
        push currentRate;
        call f;
        mov retval, eax; /* return value is in EAX register */
    }
    return retval;
}

static cwASIOError cwASIOSetSampleRate(cwASIOSampleRate sampleRate) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*)  theAsioDriver->lpVtbl->setSampleRate;
    void *p = (void*) &sampleRate;
    cwASIOError retval;
    __asm
    {
        mov ecx, theAsioDriver; /* this pointer needs to be in ECX register */
        mov eax, p;
        push [eax+04h];
        push [eax];
        call f;
        mov retval, eax; /* return value is in EAX register */
    }
    return retval;
}

static cwASIOError cwASIOGetClockSources(struct cwASIOClockSource *clocks, long *numSources) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->getClockSources;
    cwASIOError retval;
    __asm
    {
        mov ecx, theAsioDriver; /* this pointer needs to be in ECX register */
        push numSources;
        push clocks;
        call f;
        mov retval, eax; /* return value is in EAX register */
    }
    return retval;
}

static cwASIOError cwASIOSetClockSource(long reference) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->setClockSource;
    cwASIOError retval;
    __asm
    {
        mov ecx, theAsioDriver; /* this pointer needs to be in ECX register */
        push reference;
        call f;
        mov retval, eax; /* return value is in EAX register */
    }
    return retval;
}

static cwASIOError cwASIOGetSamplePosition (cwASIOSamples *sPos, cwASIOTimeStamp *tStamp) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->getSamplePosition;
    cwASIOError retval;
    __asm
    {
        mov ecx, theAsioDriver; /* this pointer needs to be in ECX register */
        push tStamp;
        push sPos;
        call f;
        mov retval, eax; /* return value is in EAX register */
    }
    return retval;
}

static cwASIOError cwASIOGetChannelInfo(struct cwASIOChannelInfo *info) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->getChannelInfo;
    cwASIOError retval;
    __asm
    {
        mov ecx, theAsioDriver; /* this pointer needs to be in ECX register */
        push info;
        call f;
        mov retval, eax; /* return value is in EAX register */
    }
    return retval;
}

static cwASIOError cwASIOCreateBuffers(struct cwASIOBufferInfo *bufferInfos, long numChannels, long bufferSize, struct cwASIOCallbacks const *callbacks) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->createBuffers;
    cwASIOError retval;
    __asm
    {
        mov ecx, theAsioDriver; /* this pointer needs to be in ECX register */
        push callbacks;
        push bufferSize;
        push numChannels;
        push bufferInfos;
        call f;
        mov retval, eax; /* return value is in EAX register */
    }
    return retval;
}

static cwASIOError cwASIODisposeBuffers(void) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->disposeBuffers;
    cwASIOError retval;
    __asm
    {
        mov ecx, theAsioDriver; /* this pointer needs to be in ECX register */
        call f;
        mov retval, eax; /* return value is in EAX register */
    }
    return retval;
}

static cwASIOError cwASIOControlPanel(void) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->controlPanel;
    cwASIOError retval;
    __asm
    {
        mov ecx, theAsioDriver; /* this pointer needs to be in ECX register */
        call f;
        mov retval, eax; /* return value is in EAX register */
    }
    return retval;
}

static cwASIOError cwASIOFuture(long selector, void *params) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->future;
    cwASIOError retval;
    __asm
    {
        mov ecx, theAsioDriver; /* this pointer needs to be in ECX register */
        push params;
        push selector;
        call f;
        mov retval, eax; /* return value is in EAX register */
    }
    return retval;
}

static cwASIOError cwASIOOutputReady(void) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->outputReady;
    cwASIOError retval;
    __asm
    {
        mov ecx, theAsioDriver; /* this pointer needs to be in ECX register */
        call f;
        mov retval, eax; /* return value is in EAX register */
    }
    return retval;
}
#elif INTPTR_MAX != INT64_MAX && defined(_WIN32) && defined(__GNUC__)
/* 32 bit Windows platform and GCC compiler */
static cwASIOError cwASIOInit(cwASIODriverInfo *info) {
    if(!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    {
        void *f = (void*) theAsioDriver->lpVtbl->init;
        void *p =         info ? info->sysRef : 0;
        cwASIOError retval;
        __asm__ __volatile__ ("pushl %3\n\t"
                              "call *%2\n\t"
                              :"=a"(retval) /* Output Operands */
                              :"c"(theAsioDriver), "r"(f), "r"(p) /* Input Operands */
                              : /* Clobbered Registers */
                             );
        if (!retval)
            return ASE_NotPresent;
    }
    if(info) {
        info->asioVersion = 2;
        {
            void *f = (void*) theAsioDriver->lpVtbl->getDriverName;
            char *p =         info->name;
            __asm
            {
                mov ecx, theAsioDriver; /* this pointer needs to be in ECX register */
                push p;
                call f;
            }
            __asm__ __volatile__ ("pushl %2\n\t"
                                  "call *%1\n\t"
                                  : /* Output Operands */
                                  :"c"(theAsioDriver), "r"(f), "r"(p) /* Input Operands */
                                  : /* Clobbered Registers */
                                 );
        }
        {
            void *f = (void*) theAsioDriver->lpVtbl->getDriverVersion;
            long retval;
            __asm__ __volatile__ ("call *%2\n\t"
                                  :"=a"(retval) /* Output Operands */
                                  :"c"(theAsioDriver), "r"(f) /* Input Operands */
                                  : /* Clobbered Registers */
                                 );
            info->driverVersion = retval;
        }
        {
            void *f = (void*) theAsioDriver->lpVtbl->getErrorMessage;
            char *p = (char*) info->errorMessage;
            __asm__ __volatile__ ("pushl %2\n\t"
                                  "call *%1\n\t"
                                  : /* Output Operands */
                                  :"c"(theAsioDriver), "r"(f), "r"(p) /* Input Operands */
                                  : /* Clobbered Registers */
                                 );
        }
    }
    return ASE_OK;
}

static cwASIOError cwASIOStart(void) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->start;
    cwASIOError retval;
    __asm__ __volatile__ ("call *%2\n\t"
                          :"=a"(retval) /* Output Operands */
                          :"c"(theAsioDriver), "r"(f) /* Input Operands */
                          : /* Clobbered Registers */
                         );
    return retval;
}

static cwASIOError cwASIOStop(void) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->stop;
    cwASIOError retval;
    __asm__ __volatile__ ("call *%2\n\t"
                          :"=a"(retval) /* Output Operands */
                          :"c"(theAsioDriver), "r"(f) /* Input Operands */
                          : /* Clobbered Registers */
                         );
    return retval;
}

static cwASIOError cwASIOGetChannels(long *numInputChannels, long *numOutputChannels) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->getChannels;
    cwASIOError retval;
    __asm__ __volatile__ ("pushl %3\n\t"
                          "pushl %4\n\t"
                          "call *%2\n\t"
                          :"=a"(retval) /* Output Operands */
                          :"c"(theAsioDriver), "r"(f), "r"(numOutputChannels), "r"(numInputChannels) /* Input Operands */
                          : /* Clobbered Registers */
                         );
    return retval;
}

static cwASIOError cwASIOGetLatencies(long *inputLatency, long *outputLatency) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->getLatencies;
    cwASIOError retval;
    __asm__ __volatile__ ("pushl %3\n\t"
                          "pushl %4\n\t"
                          "call *%2\n\t"
                          :"=a"(retval) /* Output Operands */
                          :"c"(theAsioDriver), "r"(f), "r"(outputLatency), "r"(inputLatency) /* Input Operands */
                          : /* Clobbered Registers */
                         );
    return retval;
}

static cwASIOError cwASIOGetBufferSize(long *minSize, long *maxSize, long *preferredSize, long *granularity) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->getBufferSize;
    cwASIOError retval;
    __asm__ __volatile__ ("pushl %3\n\t"
                          "pushl %4\n\t"
                          "pushl %5\n\t"
                          "pushl %6\n\t"
                          "call *%2\n\t"
                          :"=a"(retval) /* Output Operands */
                          :"c"(theAsioDriver), "r"(f), "r"(granularity), "r"(preferredSize), "r"(maxSize), "r"(minSize) /* Input Operands */
                          : /* Clobbered Registers */
                         );
    return retval;
}

static cwASIOError cwASIOCanSampleRate(cwASIOSampleRate sampleRate) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*)  theAsioDriver->lpVtbl->canSampleRate;
    void *p = (void*) &sampleRate;
    cwASIOError retval;
    __asm__ __volatile__ ("pushl "#4"(%3)\n\t"
                          "pushl (%3)\n\t"
                          "call *%2\n\t"
                          :"=a"(retval) /* Output Operands */
                          :"c"(theAsioDriver), "r"(f), "r"(p) /* Input Operands */
                          : /* Clobbered Registers */
                         );
    return retval;
}

static cwASIOError cwASIOGetSampleRate(cwASIOSampleRate *currentRate) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->getErrorMessage;
    __asm__ __volatile__ ("pushl %3\n\t"
                          "call *%2\n\t"
                          :"=a"(retval) /* Output Operands */
                          :"c"(theAsioDriver), "r"(f), "r"(currentRate) /* Input Operands */
                          : /* Clobbered Registers */
                         );
    return retval;
}

static cwASIOError cwASIOSetSampleRate(cwASIOSampleRate sampleRate) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*)  theAsioDriver->lpVtbl->setSampleRate;
    void *p = (void*) &sampleRate;
    cwASIOError retval;
    __asm__ __volatile__ ("pushl "#4"(%3)\n\t"
                          "pushl (%3)\n\t"
                          "call *%2\n\t"
                          :"=a"(retval) /* Output Operands */
                          :"c"(theAsioDriver), "r"(f), "r"(p) /* Input Operands */
                          : /* Clobbered Registers */
                         );
    return retval;
}

static cwASIOError cwASIOGetClockSources(cwASIOClockSource *clocks, long *numSources) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->getChannels;
    cwASIOError retval;
    __asm__ __volatile__ ("pushl %3\n\t"
                          "pushl %4\n\t"
                          "call *%2\n\t"
                          :"=a"(retval) /* Output Operands */
                          :"c"(theAsioDriver), "r"(f), "r"(numSources), "r"(clocks) /* Input Operands */
                          : /* Clobbered Registers */
                         );
    return retval;
}

static cwASIOError cwASIOSetClockSource(long reference) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->setClockSource;
    cwASIOError retval;
    __asm__ __volatile__ ("pushl %3\n\t"
                          "call *%2\n\t"
                          :"=a"(retval) /* Output Operands */
                          :"c"(theAsioDriver), "r"(f), "r"(reference) /* Input Operands */
                          : /* Clobbered Registers */
                         );
    return retval;
}

static cwASIOError cwASIOGetSamplePosition (cwASIOSamples *sPos, cwASIOTimeStamp *tStamp) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->getSamplePosition;
    cwASIOError retval;
    __asm__ __volatile__ ("pushl %3\n\t"
                          "pushl %4\n\t"
                          "call *%2\n\t"
                          :"=a"(retval) /* Output Operands */
                          :"c"(theAsioDriver), "r"(f), "r"(tStamp), "r"(sPos) /* Input Operands */
                          : /* Clobbered Registers */
                         );
    return retval;
}

static cwASIOError cwASIOGetChannelInfo(cwASIOChannelInfo *info) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->getChannelInfo;
    cwASIOError retval;
    __asm__ __volatile__ ("pushl %3\n\t"
                          "call *%2\n\t"
                          :"=a"(retval) /* Output Operands */
                          :"c"(theAsioDriver), "r"(f), "r"(info) /* Input Operands */
                          : /* Clobbered Registers */
                         );
    return retval;
}

static cwASIOError cwASIOCreateBuffers(cwASIOBufferInfo *bufferInfos, long numChannels, long bufferSize, cwASIOCallbacks const *callbacks) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->createBuffers;
    cwASIOError retval;
    __asm
    {
        mov ecx, theAsioDriver; /* this pointer needs to be in ECX register */
        push callbacks;
        push bufferSize;
        push numChannels;
        push bufferInfos;
        call f;
        mov retval, eax; /* return value is in EAX register */
    }
    __asm__ __volatile__ ("pushl %3\n\t"
                          "pushl %4\n\t"
                          "pushl %5\n\t"
                          "pushl %6\n\t"
                          "call *%2\n\t"
                          :"=a"(retval) /* Output Operands */
                          :"c"(theAsioDriver), "r"(f), "r"(callbacks), "r"(bufferSize), "r"(numChannels), "r"(bufferInfos) /* Input Operands */
                          : /* Clobbered Registers */
                         );
    return retval;
}

static cwASIOError cwASIODisposeBuffers(void) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->disposeBuffers;
    cwASIOError retval;
    __asm__ __volatile__ ("call *%2\n\t"
                          :"=a"(retval) /* Output Operands */
                          :"c"(theAsioDriver), "r"(f) /* Input Operands */
                          : /* Clobbered Registers */
                         );
    return retval;
}

static cwASIOError cwASIOControlPanel(void) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->controlPanel;
    cwASIOError retval;
    __asm__ __volatile__ ("call *%2\n\t"
                          :"=a"(retval) /* Output Operands */
                          :"c"(theAsioDriver), "r"(f) /* Input Operands */
                          : /* Clobbered Registers */
                         );
    return retval;
}

static cwASIOError cwASIOFuture(long selector, void *params) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->future;
    cwASIOError retval;
    __asm__ __volatile__ ("pushl %3\n\t"
                          "pushl %4\n\t"
                          "call *%2\n\t"
                          :"=a"(retval) /* Output Operands */
                          :"c"(theAsioDriver), "r"(f), "r"(params), "r"(selector) /* Input Operands */
                          : /* Clobbered Registers */
                         );
    return retval;
}

static cwASIOError cwASIOOutputReady(void) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    void *f = (void*) theAsioDriver->lpVtbl->outputReady;
    cwASIOError retval;
    __asm__ __volatile__ ("call *%2\n\t"
                          :"=a"(retval) /* Output Operands */
                          :"c"(theAsioDriver), "r"(f) /* Input Operands */
                          : /* Clobbered Registers */
                         );
    return retval;
}
#else
static cwASIOError cwASIOInit(struct cwASIODriverInfo *info) {
    if(!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    if (!theAsioDriver->lpVtbl->init(theAsioDriver, info ? info->sysRef : 0))
        return ASE_NotPresent;
    if(info) {
        info->asioVersion = 2;
        theAsioDriver->lpVtbl->getDriverName(theAsioDriver, info->name);
        info->driverVersion = theAsioDriver->lpVtbl->getDriverVersion(theAsioDriver);
        theAsioDriver->lpVtbl->getErrorMessage(theAsioDriver, info->errorMessage);
    }
    return ASE_OK;
}

static cwASIOError cwASIOStart(void) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    return theAsioDriver->lpVtbl->start(theAsioDriver);
}

static cwASIOError cwASIOStop(void) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    return theAsioDriver->lpVtbl->stop(theAsioDriver);
}

static cwASIOError cwASIOGetChannels(long *numInputChannels, long *numOutputChannels) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    return theAsioDriver->lpVtbl->getChannels(theAsioDriver, numInputChannels, numOutputChannels);
}

static cwASIOError cwASIOGetLatencies(long *inputLatency, long *outputLatency) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    return theAsioDriver->lpVtbl->getLatencies(theAsioDriver, inputLatency, outputLatency);
}

static cwASIOError cwASIOGetBufferSize(long *minSize, long *maxSize, long *preferredSize, long *granularity) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    return theAsioDriver->lpVtbl->getBufferSize(theAsioDriver, minSize, maxSize, preferredSize, granularity);
}

static cwASIOError cwASIOCanSampleRate(cwASIOSampleRate sampleRate) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    return theAsioDriver->lpVtbl->canSampleRate(theAsioDriver, sampleRate);
}

static cwASIOError cwASIOGetSampleRate(cwASIOSampleRate *currentRate) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    return theAsioDriver->lpVtbl->getSampleRate(theAsioDriver, currentRate);
}

static cwASIOError cwASIOSetSampleRate(cwASIOSampleRate sampleRate) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    return theAsioDriver->lpVtbl->setSampleRate(theAsioDriver, sampleRate);
}

static cwASIOError cwASIOGetClockSources(struct cwASIOClockSource *clocks, long *numSources) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    return theAsioDriver->lpVtbl->getClockSources(theAsioDriver, clocks, numSources);
}

static cwASIOError cwASIOSetClockSource(long reference) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    return theAsioDriver->lpVtbl->setClockSource(theAsioDriver, reference);
}

static cwASIOError cwASIOGetSamplePosition (cwASIOSamples *sPos, cwASIOTimeStamp *tStamp) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    return theAsioDriver->lpVtbl->getSamplePosition(theAsioDriver, sPos, tStamp);
}

static cwASIOError cwASIOGetChannelInfo(struct cwASIOChannelInfo *info) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    return theAsioDriver->lpVtbl->getChannelInfo(theAsioDriver, info);
}

static cwASIOError cwASIOCreateBuffers(struct cwASIOBufferInfo *bufferInfos, long numChannels, long bufferSize, struct cwASIOCallbacks const *callbacks) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    return theAsioDriver->lpVtbl->createBuffers(theAsioDriver, bufferInfos, numChannels, bufferSize, callbacks);
}

static cwASIOError cwASIODisposeBuffers(void) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    return theAsioDriver->lpVtbl->disposeBuffers(theAsioDriver);
}

static cwASIOError cwASIOControlPanel(void) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    return theAsioDriver->lpVtbl->controlPanel(theAsioDriver);
}

static cwASIOError cwASIOFuture(long selector, void *params) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    return theAsioDriver->lpVtbl->future(theAsioDriver, selector, params);
}

static cwASIOError cwASIOOutputReady(void) {
    if (!theAsioDriver || !theAsioDriver->lpVtbl)
        return ASE_NotPresent;
    return theAsioDriver->lpVtbl->outputReady(theAsioDriver);
}
#endif
/* end of cwASIO convenience functions */

static bool cwAsioCallback( void *context, char const *name, char const *clsid, char const *desc )
{
    if (!context)
        return false;

    CwAsioDriverInfos *cwAsioDriverInfos = ( CwAsioDriverInfos* ) context;

    CwAsioDriverInfo *driverInfo = &cwAsioDriverInfos->asioDriverInfos[cwAsioDriverInfos->size++];
    assert(driverInfo);
    memset( driverInfo, 0, sizeof(CwAsioDriverInfo) );
    assert( sizeof(driverInfo->base.name) > 0 );
    strncpy(driverInfo->base.name, name, sizeof(driverInfo->base.name ) - 1U );
    assert( sizeof(driverInfo->clsid) > 0 );
    strncpy(driverInfo->clsid, clsid, sizeof(driverInfo->clsid) - 1U );
    assert( sizeof(driverInfo->desc) > 0 );
    strncpy(driverInfo->desc, desc, sizeof(driverInfo->desc) - 1U );

    return true;
}

static long getDriverNames( CwAsioDriverInfos *cwAsioDriverInfos, char **names, long maxDrivers )
{
    memset(cwAsioDriverInfos, 0, sizeof(CwAsioDriverInfos));
    int result = cwASIOenumerate( &cwAsioCallback, cwAsioDriverInfos);
    if ( result != 0 )
    {
        return 0;
    }

    long driverCount = 0;
    for ( size_t s = 0; s < cwAsioDriverInfos->size; ++s )
    {
        if ( driverCount >= maxDrivers )
            break;

        if ( names )
            if ( cwAsioDriverInfos->asioDriverInfos[s].desc && *cwAsioDriverInfos->asioDriverInfos[s].desc != '\0' )
                strcpy( names[driverCount], cwAsioDriverInfos->asioDriverInfos[s].desc);
            else
                strcpy( names[driverCount], cwAsioDriverInfos->asioDriverInfos[s].base.name);
        driverCount++;
    }

    return driverCount;
}

/*
    Retrieve <driverCount> driver names from ASIO, returned in a char**
    allocated in <group>.
*/
static char **GetCwAsioDriverNames( PaCwAsioHostApiRepresentation *cwAsioHostApi, PaUtilAllocationGroup *group, long driverCount )
{
    char **result = 0;
    int i;

    result =(char**)PaUtil_GroupAllocateZeroInitializedMemory(
            group, sizeof(char*) * driverCount );
    if( !result )
        goto error;

    result[0] = (char*)PaUtil_GroupAllocateZeroInitializedMemory(
            group, 32 * driverCount );
    if( !result[0] )
        goto error;

    for( i=0; i<driverCount; ++i )
        result[i] = result[0] + (32 * i);

    getDriverNames( &cwAsioHostApi->driverInfos, result, driverCount );

    return result;

error:
    if ( result )
        PaUtil_GroupFreeMemory( group, result );

    return result;
}


static PaSampleFormat AsioSampleTypeToPaNativeSampleFormat(cwASIOSampleType type)
{
    switch (type) {
        case ASIOSTInt16MSB:
        case ASIOSTInt16LSB:
                return paInt16;

        case ASIOSTFloat32MSB:
        case ASIOSTFloat32LSB:
        case ASIOSTFloat64MSB:
        case ASIOSTFloat64LSB:
                return paFloat32;

        case ASIOSTInt32MSB:
        case ASIOSTInt32LSB:
        case ASIOSTInt32MSB16:
        case ASIOSTInt32LSB16:
        case ASIOSTInt32MSB18:
        case ASIOSTInt32MSB20:
        case ASIOSTInt32MSB24:
        case ASIOSTInt32LSB18:
        case ASIOSTInt32LSB20:
        case ASIOSTInt32LSB24:
                return paInt32;

        case ASIOSTInt24MSB:
        case ASIOSTInt24LSB:
                return paInt24;

        default:
                return paCustomFormat;
    }
}

void CwAsioSampleTypeLOG(cwASIOSampleType type)
{
    switch (type) {
        case ASIOSTInt16MSB:  PA_DEBUG(("ASIOSTInt16MSB\n"));  break;
        case ASIOSTInt16LSB:  PA_DEBUG(("ASIOSTInt16LSB\n"));  break;
        case ASIOSTFloat32MSB:PA_DEBUG(("ASIOSTFloat32MSB\n"));break;
        case ASIOSTFloat32LSB:PA_DEBUG(("ASIOSTFloat32LSB\n"));break;
        case ASIOSTFloat64MSB:PA_DEBUG(("ASIOSTFloat64MSB\n"));break;
        case ASIOSTFloat64LSB:PA_DEBUG(("ASIOSTFloat64LSB\n"));break;
        case ASIOSTInt32MSB:  PA_DEBUG(("ASIOSTInt32MSB\n"));  break;
        case ASIOSTInt32LSB:  PA_DEBUG(("ASIOSTInt32LSB\n"));  break;
        case ASIOSTInt32MSB16:PA_DEBUG(("ASIOSTInt32MSB16\n"));break;
        case ASIOSTInt32LSB16:PA_DEBUG(("ASIOSTInt32LSB16\n"));break;
        case ASIOSTInt32MSB18:PA_DEBUG(("ASIOSTInt32MSB18\n"));break;
        case ASIOSTInt32MSB20:PA_DEBUG(("ASIOSTInt32MSB20\n"));break;
        case ASIOSTInt32MSB24:PA_DEBUG(("ASIOSTInt32MSB24\n"));break;
        case ASIOSTInt32LSB18:PA_DEBUG(("ASIOSTInt32LSB18\n"));break;
        case ASIOSTInt32LSB20:PA_DEBUG(("ASIOSTInt32LSB20\n"));break;
        case ASIOSTInt32LSB24:PA_DEBUG(("ASIOSTInt32LSB24\n"));break;
        case ASIOSTInt24MSB:  PA_DEBUG(("ASIOSTInt24MSB\n"));  break;
        case ASIOSTInt24LSB:  PA_DEBUG(("ASIOSTInt24LSB\n"));  break;
        default:              PA_DEBUG(("Custom Format%d\n",type));break;

    }
}

static int BytesPerAsioSample( cwASIOSampleType sampleType )
{
    switch (sampleType) {
        case ASIOSTInt16MSB:
        case ASIOSTInt16LSB:
            return 2;

        case ASIOSTFloat64MSB:
        case ASIOSTFloat64LSB:
            return 8;

        case ASIOSTFloat32MSB:
        case ASIOSTFloat32LSB:
        case ASIOSTInt32MSB:
        case ASIOSTInt32LSB:
        case ASIOSTInt32MSB16:
        case ASIOSTInt32LSB16:
        case ASIOSTInt32MSB18:
        case ASIOSTInt32MSB20:
        case ASIOSTInt32MSB24:
        case ASIOSTInt32LSB18:
        case ASIOSTInt32LSB20:
        case ASIOSTInt32LSB24:
            return 4;

        case ASIOSTInt24MSB:
        case ASIOSTInt24LSB:
            return 3;

        default:
            return 0;
    }
}


static void Swap16( void *buffer, long shift, long count )
{
    unsigned short *p = (unsigned short*)buffer;
    unsigned short temp;
    (void) shift; /* unused parameter */

    while( count-- )
    {
        temp = *p;
        *p++ = (unsigned short)((temp<<8) | (temp>>8));
    }
}

static void Swap24( void *buffer, long shift, long count )
{
    unsigned char *p = (unsigned char*)buffer;
    unsigned char temp;
    (void) shift; /* unused parameter */

    while( count-- )
    {
        temp = *p;
        *p = *(p+2);
        *(p+2) = temp;
        p += 3;
    }
}

#define PA_SWAP32_( x ) ((x>>24) | ((x>>8)&0xFF00) | ((x<<8)&0xFF0000) | (x<<24));

static void Swap32( void *buffer, long shift, long count )
{
    unsigned long *p = (unsigned long*)buffer;
    unsigned long temp;
    (void) shift; /* unused parameter */

    while( count-- )
    {
        temp = *p;
        *p++ = PA_SWAP32_( temp);
    }
}

static void SwapShiftLeft32( void *buffer, long shift, long count )
{
    unsigned long *p = (unsigned long*)buffer;
    unsigned long temp;

    while( count-- )
    {
        temp = *p;
        temp = PA_SWAP32_( temp);
        *p++ = temp << shift;
    }
}

static void ShiftRightSwap32( void *buffer, long shift, long count )
{
    unsigned long *p = (unsigned long*)buffer;
    unsigned long temp;

    while( count-- )
    {
        temp = *p >> shift;
        *p++ = PA_SWAP32_( temp);
    }
}

static void ShiftLeft32( void *buffer, long shift, long count )
{
    unsigned long *p = (unsigned long*)buffer;
    unsigned long temp;

    while( count-- )
    {
        temp = *p;
        *p++ = temp << shift;
    }
}

static void ShiftRight32( void *buffer, long shift, long count )
{
    unsigned long *p = (unsigned long*)buffer;
    unsigned long temp;

    while( count-- )
    {
        temp = *p;
        *p++ = temp >> shift;
    }
}

#define PA_SWAP_( x, y ) temp=x; x = y; y = temp;

static void Swap64ConvertFloat64ToFloat32( void *buffer, long shift, long count )
{
    double *in = (double*)buffer;
    float *out = (float*)buffer;
    unsigned char *p;
    unsigned char temp;
    (void) shift; /* unused parameter */

    while( count-- )
    {
        p = (unsigned char*)in;
        PA_SWAP_( p[0], p[7] );
        PA_SWAP_( p[1], p[6] );
        PA_SWAP_( p[2], p[5] );
        PA_SWAP_( p[3], p[4] );

        *out++ = (float) (*in++);
    }
}

static void ConvertFloat64ToFloat32( void *buffer, long shift, long count )
{
    double *in = (double*)buffer;
    float *out = (float*)buffer;
    (void) shift; /* unused parameter */

    while( count-- )
        *out++ = (float) (*in++);
}

static void ConvertFloat32ToFloat64Swap64( void *buffer, long shift, long count )
{
    float *in = ((float*)buffer) + (count-1);
    double *out = ((double*)buffer) + (count-1);
    unsigned char *p;
    unsigned char temp;
    (void) shift; /* unused parameter */

    while( count-- )
    {
        *out = *in--;

        p = (unsigned char*)out;
        PA_SWAP_( p[0], p[7] );
        PA_SWAP_( p[1], p[6] );
        PA_SWAP_( p[2], p[5] );
        PA_SWAP_( p[3], p[4] );

        out--;
    }
}

static void ConvertFloat32ToFloat64( void *buffer, long shift, long count )
{
    float *in = ((float*)buffer) + (count-1);
    double *out = ((double*)buffer) + (count-1);
    (void) shift; /* unused parameter */

    while( count-- )
        *out-- = *in--;
}

#ifdef WINDOWS
#undef PA_MSB_IS_NATIVE_
#define PA_LSB_IS_NATIVE_
#endif

typedef void PaCwAsioBufferConverter( void *, long, long );

static void SelectAsioToPaConverter( cwASIOSampleType type, PaCwAsioBufferConverter **converter, long *shift )
{
    *shift = 0;
    *converter = 0;

    switch (type) {
        case ASIOSTInt16MSB:
            /* dest: paInt16, no conversion necessary, possible byte swap*/
            #ifdef PA_LSB_IS_NATIVE_
                *converter = Swap16;
            #endif
            break;
        case ASIOSTInt16LSB:
            /* dest: paInt16, no conversion necessary, possible byte swap*/
            #ifdef PA_MSB_IS_NATIVE_
                *converter = Swap16;
            #endif
            break;
        case ASIOSTFloat32MSB:
            /* dest: paFloat32, no conversion necessary, possible byte swap*/
            #ifdef PA_LSB_IS_NATIVE_
                *converter = Swap32;
            #endif
            break;
        case ASIOSTFloat32LSB:
            /* dest: paFloat32, no conversion necessary, possible byte swap*/
            #ifdef PA_MSB_IS_NATIVE_
                *converter = Swap32;
            #endif
            break;
        case ASIOSTFloat64MSB:
            /* dest: paFloat32, in-place conversion to/from float32, possible byte swap*/
            #ifdef PA_LSB_IS_NATIVE_
                *converter = Swap64ConvertFloat64ToFloat32;
            #else
                *converter = ConvertFloat64ToFloat32;
            #endif
            break;
        case ASIOSTFloat64LSB:
            /* dest: paFloat32, in-place conversion to/from float32, possible byte swap*/
            #ifdef PA_MSB_IS_NATIVE_
                *converter = Swap64ConvertFloat64ToFloat32;
            #else
                *converter = ConvertFloat64ToFloat32;
            #endif
            break;
        case ASIOSTInt32MSB:
            /* dest: paInt32, no conversion necessary, possible byte swap */
            #ifdef PA_LSB_IS_NATIVE_
                *converter = Swap32;
            #endif
            break;
        case ASIOSTInt32LSB:
            /* dest: paInt32, no conversion necessary, possible byte swap */
            #ifdef PA_MSB_IS_NATIVE_
                *converter = Swap32;
            #endif
            break;
        case ASIOSTInt32MSB16:
            /* dest: paInt32, 16 bit shift, possible byte swap */
            #ifdef PA_LSB_IS_NATIVE_
                *converter = SwapShiftLeft32;
            #else
                *converter = ShiftLeft32;
            #endif
            *shift = 16;
            break;
        case ASIOSTInt32MSB18:
            /* dest: paInt32, 14 bit shift, possible byte swap */
            #ifdef PA_LSB_IS_NATIVE_
                *converter = SwapShiftLeft32;
            #else
                *converter = ShiftLeft32;
            #endif
            *shift = 14;
            break;
        case ASIOSTInt32MSB20:
            /* dest: paInt32, 12 bit shift, possible byte swap */
            #ifdef PA_LSB_IS_NATIVE_
                *converter = SwapShiftLeft32;
            #else
                *converter = ShiftLeft32;
            #endif
            *shift = 12;
            break;
        case ASIOSTInt32MSB24:
            /* dest: paInt32, 8 bit shift, possible byte swap */
            #ifdef PA_LSB_IS_NATIVE_
                *converter = SwapShiftLeft32;
            #else
                *converter = ShiftLeft32;
            #endif
            *shift = 8;
            break;
        case ASIOSTInt32LSB16:
            /* dest: paInt32, 16 bit shift, possible byte swap */
            #ifdef PA_MSB_IS_NATIVE_
                *converter = SwapShiftLeft32;
            #else
                *converter = ShiftLeft32;
            #endif
            *shift = 16;
            break;
        case ASIOSTInt32LSB18:
            /* dest: paInt32, 14 bit shift, possible byte swap */
            #ifdef PA_MSB_IS_NATIVE_
                *converter = SwapShiftLeft32;
            #else
                *converter = ShiftLeft32;
            #endif
            *shift = 14;
            break;
        case ASIOSTInt32LSB20:
            /* dest: paInt32, 12 bit shift, possible byte swap */
            #ifdef PA_MSB_IS_NATIVE_
                *converter = SwapShiftLeft32;
            #else
                *converter = ShiftLeft32;
            #endif
            *shift = 12;
            break;
        case ASIOSTInt32LSB24:
            /* dest: paInt32, 8 bit shift, possible byte swap */
            #ifdef PA_MSB_IS_NATIVE_
                *converter = SwapShiftLeft32;
            #else
                *converter = ShiftLeft32;
            #endif
            *shift = 8;
            break;
        case ASIOSTInt24MSB:
            /* dest: paInt24, no conversion necessary, possible byte swap */
            #ifdef PA_LSB_IS_NATIVE_
                *converter = Swap24;
            #endif
            break;
        case ASIOSTInt24LSB:
            /* dest: paInt24, no conversion necessary, possible byte swap */
            #ifdef PA_MSB_IS_NATIVE_
                *converter = Swap24;
            #endif
            break;
    }
}


static void SelectPaToAsioConverter( cwASIOSampleType type, PaCwAsioBufferConverter **converter, long *shift )
{
    *shift = 0;
    *converter = 0;

    switch (type) {
        case ASIOSTInt16MSB:
            /* src: paInt16, no conversion necessary, possible byte swap*/
            #ifdef PA_LSB_IS_NATIVE_
                *converter = Swap16;
            #endif
            break;
        case ASIOSTInt16LSB:
            /* src: paInt16, no conversion necessary, possible byte swap*/
            #ifdef PA_MSB_IS_NATIVE_
                *converter = Swap16;
            #endif
            break;
        case ASIOSTFloat32MSB:
            /* src: paFloat32, no conversion necessary, possible byte swap*/
            #ifdef PA_LSB_IS_NATIVE_
                *converter = Swap32;
            #endif
            break;
        case ASIOSTFloat32LSB:
            /* src: paFloat32, no conversion necessary, possible byte swap*/
            #ifdef PA_MSB_IS_NATIVE_
                *converter = Swap32;
            #endif
            break;
        case ASIOSTFloat64MSB:
            /* src: paFloat32, in-place conversion to/from float32, possible byte swap*/
            #ifdef PA_LSB_IS_NATIVE_
                *converter = ConvertFloat32ToFloat64Swap64;
            #else
                *converter = ConvertFloat32ToFloat64;
            #endif
            break;
        case ASIOSTFloat64LSB:
            /* src: paFloat32, in-place conversion to/from float32, possible byte swap*/
            #ifdef PA_MSB_IS_NATIVE_
                *converter = ConvertFloat32ToFloat64Swap64;
            #else
                *converter = ConvertFloat32ToFloat64;
            #endif
            break;
        case ASIOSTInt32MSB:
            /* src: paInt32, no conversion necessary, possible byte swap */
            #ifdef PA_LSB_IS_NATIVE_
                *converter = Swap32;
            #endif
            break;
        case ASIOSTInt32LSB:
            /* src: paInt32, no conversion necessary, possible byte swap */
            #ifdef PA_MSB_IS_NATIVE_
                *converter = Swap32;
            #endif
            break;
        case ASIOSTInt32MSB16:
            /* src: paInt32, 16 bit shift, possible byte swap */
            #ifdef PA_LSB_IS_NATIVE_
                *converter = ShiftRightSwap32;
            #else
                *converter = ShiftRight32;
            #endif
            *shift = 16;
            break;
        case ASIOSTInt32MSB18:
            /* src: paInt32, 14 bit shift, possible byte swap */
            #ifdef PA_LSB_IS_NATIVE_
                *converter = ShiftRightSwap32;
            #else
                *converter = ShiftRight32;
            #endif
            *shift = 14;
            break;
        case ASIOSTInt32MSB20:
            /* src: paInt32, 12 bit shift, possible byte swap */
            #ifdef PA_LSB_IS_NATIVE_
                *converter = ShiftRightSwap32;
            #else
                *converter = ShiftRight32;
            #endif
            *shift = 12;
            break;
        case ASIOSTInt32MSB24:
            /* src: paInt32, 8 bit shift, possible byte swap */
            #ifdef PA_LSB_IS_NATIVE_
                *converter = ShiftRightSwap32;
            #else
                *converter = ShiftRight32;
            #endif
            *shift = 8;
            break;
        case ASIOSTInt32LSB16:
            /* src: paInt32, 16 bit shift, possible byte swap */
            #ifdef PA_MSB_IS_NATIVE_
                *converter = ShiftRightSwap32;
            #else
                *converter = ShiftRight32;
            #endif
            *shift = 16;
            break;
        case ASIOSTInt32LSB18:
            /* src: paInt32, 14 bit shift, possible byte swap */
            #ifdef PA_MSB_IS_NATIVE_
                *converter = ShiftRightSwap32;
            #else
                *converter = ShiftRight32;
            #endif
            *shift = 14;
            break;
        case ASIOSTInt32LSB20:
            /* src: paInt32, 12 bit shift, possible byte swap */
            #ifdef PA_MSB_IS_NATIVE_
                *converter = ShiftRightSwap32;
            #else
                *converter = ShiftRight32;
            #endif
            *shift = 12;
            break;
        case ASIOSTInt32LSB24:
            /* src: paInt32, 8 bit shift, possible byte swap */
            #ifdef PA_MSB_IS_NATIVE_
                *converter = ShiftRightSwap32;
            #else
                *converter = ShiftRight32;
            #endif
            *shift = 8;
            break;
        case ASIOSTInt24MSB:
            /* src: paInt24, no conversion necessary, possible byte swap */
            #ifdef PA_LSB_IS_NATIVE_
                *converter = Swap24;
            #endif
            break;
        case ASIOSTInt24LSB:
            /* src: paInt24, no conversion necessary, possible byte swap */
            #ifdef PA_MSB_IS_NATIVE_
                *converter = Swap24;
            #endif
            break;
    }
}


typedef struct PaCwAsioDeviceInfo
{
    PaDeviceInfo commonDeviceInfo;
    long minBufferSize;
    long maxBufferSize;
    long preferredBufferSize;
    long bufferGranularity;

    struct cwASIOChannelInfo *asioChannelInfos;
}
PaCwAsioDeviceInfo;


PaError PaCwAsio_GetAvailableBufferSizes( PaDeviceIndex device,
        long *minBufferSizeFrames, long *maxBufferSizeFrames, long *preferredBufferSizeFrames, long *granularity )
{
    PaError result;
    PaUtilHostApiRepresentation *hostApi;
    PaDeviceIndex hostApiDevice;

    result = PaUtil_GetHostApiRepresentation( &hostApi, paCwASIO );

    if( result == paNoError )
    {
        result = PaUtil_DeviceIndexToHostApiDeviceIndex( &hostApiDevice, device, hostApi );

        if( result == paNoError )
        {
            PaCwAsioDeviceInfo *asioDeviceInfo =
                    (PaCwAsioDeviceInfo*)hostApi->deviceInfos[hostApiDevice];

            *minBufferSizeFrames = asioDeviceInfo->minBufferSize;
            *maxBufferSizeFrames = asioDeviceInfo->maxBufferSize;
            *preferredBufferSizeFrames = asioDeviceInfo->preferredBufferSize;
            *granularity = asioDeviceInfo->bufferGranularity;
        }
    }

    return result;
}

/* Unload whatever we loaded in LoadAsioDriver().
*/
static void UnloadAsioDriver( void )
{
    cwASIOExit();
    cwASIOUnload();
}

static PaError getDriverClsid( PaCwAsioHostApiRepresentation *cwAsioHostApi, char const *name, char *clsid, size_t clsidSize )
{
    if (!clsid || clsidSize == 0)
        return paInsufficientMemory;
    *clsid = '\0';
    if (cwAsioHostApi->driverInfos.size == 0)
        return paNoError;
    if (clsidSize < sizeof(cwAsioHostApi->driverInfos.asioDriverInfos[0].clsid))
        return paInsufficientMemory;

    for (size_t s = 0; s < cwAsioHostApi->driverInfos.size; ++s)
    {
        if (strncmp(cwAsioHostApi->driverInfos.asioDriverInfos[s].base.name, name, sizeof(cwAsioHostApi->driverInfos.asioDriverInfos[s].base.name)) == 0)
        {
            strncpy(clsid, cwAsioHostApi->driverInfos.asioDriverInfos[s].clsid, sizeof(cwAsioHostApi->driverInfos.asioDriverInfos[s].clsid));
            clsid[--clsidSize] = '\0';
            return paNoError;
        }
    }

    for (size_t s = 0; s < cwAsioHostApi->driverInfos.size; ++s)
    {
        if (strncmp(cwAsioHostApi->driverInfos.asioDriverInfos[s].desc, name, sizeof(cwAsioHostApi->driverInfos.asioDriverInfos[s].desc)) == 0)
        {
            strncpy(clsid, cwAsioHostApi->driverInfos.asioDriverInfos[s].clsid, sizeof(cwAsioHostApi->driverInfos.asioDriverInfos[s].clsid));
            clsid[--clsidSize] = '\0';
            return paNoError;
        }
    }

    return paNoError;
}

/*
    load the asio driver named by <driverName> and return statistics about
    the driver in info. If no error occurred, the driver will remain open
    and must be closed by the called by calling UnloadAsioDriver() - if an error
    is returned the driver will already be unloaded.
*/
static PaError LoadAsioDriver( PaCwAsioHostApiRepresentation *cwAsioHostApi, const char *driverName,
        PaCwAsioDriverInfo *driverInfo, void *systemSpecific )
{
    PaError result = paNoError;
    cwASIOError asioError;
    int asioIsLoaded = 0;
    int asioIsInitialized = 0;
    char clsid[64]; *clsid = '\0';
    PaError paError = getDriverClsid( cwAsioHostApi, driverName, clsid, sizeof(clsid) );
    if (paError != paNoError || *clsid == '\0')
    {
        result = paUnanticipatedHostError;
        PA_CWASIO_SET_LAST_HOST_ERROR(0, "Failed to gett CLSID for ASIO driver");
        goto error;
    }

    if( cwASIOLoad( clsid ) != ASE_OK )
    {
        result = paUnanticipatedHostError;
        PA_CWASIO_SET_LAST_HOST_ERROR( 0, "Failed to load ASIO driver" );
        goto error;
    }
    else
    {
        asioIsLoaded = 1;
    }

    memset( &driverInfo->cwAsioDriverInfo, 0, sizeof(CwAsioDriverInfo) );
    driverInfo->cwAsioDriverInfo.base.asioVersion = 2;
    driverInfo->cwAsioDriverInfo.base.sysRef = systemSpecific;
    if( (asioError = cwASIOInit( &driverInfo->cwAsioDriverInfo.base )) != ASE_OK )
    {
        result = paUnanticipatedHostError;
        PA_CWASIO_SET_LAST_ASIO_ERROR( asioError );
        goto error;
    }
    else
    {
        asioIsInitialized = 1;
    }

    if( (asioError = cwASIOGetChannels(&driverInfo->inputChannelCount,
            &driverInfo->outputChannelCount)) != ASE_OK )
    {
        result = paUnanticipatedHostError;
        PA_CWASIO_SET_LAST_ASIO_ERROR( asioError );
        goto error;
    }

    if( (asioError = cwASIOGetBufferSize(&driverInfo->bufferMinSize,
            &driverInfo->bufferMaxSize, &driverInfo->bufferPreferredSize,
            &driverInfo->bufferGranularity)) != ASE_OK )
    {
        result = paUnanticipatedHostError;
        PA_CWASIO_SET_LAST_ASIO_ERROR( asioError );
        goto error;
    }

    if( cwASIOOutputReady() == ASE_OK )
        driverInfo->postOutput = true;
    else
        driverInfo->postOutput = false;

    return result;

error:
    if (asioIsInitialized)
    {
        cwASIOExit();
    }
    if (asioIsLoaded)
    {
        cwASIOUnload();
    }

    return result;
}


#define PA_DEFAULTSAMPLERATESEARCHORDER_COUNT_     13   /* must be the same number of elements as in the array below */
static cwASIOSampleRate defaultSampleRateSearchOrder_[]
     = {44100.0, 48000.0, 32000.0, 24000.0, 22050.0, 88200.0, 96000.0,
        192000.0, 16000.0, 12000.0, 11025.0, 9600.0, 8000.0 };


static PaError InitPaDeviceInfoFromAsioDriver( PaCwAsioHostApiRepresentation *cwAsioHostApi,
        const char *driverName, int driverIndex,
        PaDeviceInfo *deviceInfo, PaCwAsioDeviceInfo *cwAsioDeviceInfo )
{
    PaError result = paNoError;

    /* Due to the headless design of the ASIO API, drivers are free to write over data given to them (like M-Audio
       drivers f.i.). This is an attempt to overcome that. */
    union _tag_local {
        PaCwAsioDriverInfo info;
        char _padding[4096];
    } paCwAsioDriver;

    cwAsioDeviceInfo->asioChannelInfos = 0; /* we check this below to handle error cleanup */

    result = LoadAsioDriver( cwAsioHostApi, driverName, &paCwAsioDriver.info, cwAsioHostApi->systemSpecific );
    if( result == paNoError )
    {
        PA_DEBUG(("PaAsio_Initialize: drv:%d name = %s\n",  driverIndex,deviceInfo->name));
        PA_DEBUG(("PaAsio_Initialize: drv:%d inputChannels       = %d\n", driverIndex, paAsioDriver.info.inputChannelCount));
        PA_DEBUG(("PaAsio_Initialize: drv:%d outputChannels      = %d\n", driverIndex, paAsioDriver.info.outputChannelCount));
        PA_DEBUG(("PaAsio_Initialize: drv:%d bufferMinSize       = %d\n", driverIndex, paAsioDriver.info.bufferMinSize));
        PA_DEBUG(("PaAsio_Initialize: drv:%d bufferMaxSize       = %d\n", driverIndex, paAsioDriver.info.bufferMaxSize));
        PA_DEBUG(("PaAsio_Initialize: drv:%d bufferPreferredSize = %d\n", driverIndex, paAsioDriver.info.bufferPreferredSize));
        PA_DEBUG(("PaAsio_Initialize: drv:%d bufferGranularity   = %d\n", driverIndex, paAsioDriver.info.bufferGranularity));

        deviceInfo->maxInputChannels  = paCwAsioDriver.info.inputChannelCount;
        deviceInfo->maxOutputChannels = paCwAsioDriver.info.outputChannelCount;

        deviceInfo->defaultSampleRate = 0.;
        bool foundDefaultSampleRate = false;
        for( int j=0; j < PA_DEFAULTSAMPLERATESEARCHORDER_COUNT_; ++j )
        {
            cwASIOError asioError = cwASIOCanSampleRate( defaultSampleRateSearchOrder_[j] );
            if( asioError != ASE_NoClock && asioError != ASE_NotPresent )
            {
                deviceInfo->defaultSampleRate = defaultSampleRateSearchOrder_[j];
                foundDefaultSampleRate = true;
                break;
            }
        }

        PA_DEBUG(("PaAsio_Initialize: drv:%d defaultSampleRate = %f\n", driverIndex, deviceInfo->defaultSampleRate));

        if( foundDefaultSampleRate ){

            /* calculate default latency values from bufferPreferredSize
                for default low latency, and bufferMaxSize
                for default high latency.
                use the default sample rate to convert from samples to
                seconds. Without knowing what sample rate the user will
                use this is the best we can do.
            */

            double defaultLowLatency =
                    paCwAsioDriver.info.bufferPreferredSize / deviceInfo->defaultSampleRate;

            deviceInfo->defaultLowInputLatency = defaultLowLatency;
            deviceInfo->defaultLowOutputLatency = defaultLowLatency;

            double defaultHighLatency =
                    paCwAsioDriver.info.bufferMaxSize / deviceInfo->defaultSampleRate;

            if( defaultHighLatency < defaultLowLatency )
                defaultHighLatency = defaultLowLatency; /* just in case the driver returns something strange */

            deviceInfo->defaultHighInputLatency = defaultHighLatency;
            deviceInfo->defaultHighOutputLatency = defaultHighLatency;

        }else{

            deviceInfo->defaultLowInputLatency = 0.;
            deviceInfo->defaultLowOutputLatency = 0.;
            deviceInfo->defaultHighInputLatency = 0.;
            deviceInfo->defaultHighOutputLatency = 0.;
        }

        PA_DEBUG(("PaAsio_Initialize: drv:%d defaultLowInputLatency = %f\n", driverIndex, deviceInfo->defaultLowInputLatency));
        PA_DEBUG(("PaAsio_Initialize: drv:%d defaultLowOutputLatency = %f\n", driverIndex, deviceInfo->defaultLowOutputLatency));
        PA_DEBUG(("PaAsio_Initialize: drv:%d defaultHighInputLatency = %f\n", driverIndex, deviceInfo->defaultHighInputLatency));
        PA_DEBUG(("PaAsio_Initialize: drv:%d defaultHighOutputLatency = %f\n", driverIndex, deviceInfo->defaultHighOutputLatency));

        cwAsioDeviceInfo->minBufferSize = paCwAsioDriver.info.bufferMinSize;
        cwAsioDeviceInfo->maxBufferSize = paCwAsioDriver.info.bufferMaxSize;
        cwAsioDeviceInfo->preferredBufferSize = paCwAsioDriver.info.bufferPreferredSize;
        cwAsioDeviceInfo->bufferGranularity = paCwAsioDriver.info.bufferGranularity;


        cwAsioDeviceInfo->asioChannelInfos = (struct cwASIOChannelInfo*)PaUtil_GroupAllocateZeroInitializedMemory(
                cwAsioHostApi->allocations,
                sizeof(struct cwASIOChannelInfo) * (deviceInfo->maxInputChannels
                        + deviceInfo->maxOutputChannels) );
        if( !cwAsioDeviceInfo->asioChannelInfos )
        {
            result = paInsufficientMemory;
            goto error_unload;
        }

        int a;

        for( a=0; a < deviceInfo->maxInputChannels; ++a ){
            cwAsioDeviceInfo->asioChannelInfos[a].channel = a;
            cwAsioDeviceInfo->asioChannelInfos[a].isInput = ASIOTrue;
            cwASIOError asioError = cwASIOGetChannelInfo( &cwAsioDeviceInfo->asioChannelInfos[a] );
            if( asioError != ASE_OK )
            {
                result = paUnanticipatedHostError;
                PA_CWASIO_SET_LAST_ASIO_ERROR( asioError );
                goto error_unload;
            }
        }

        for( a=0; a < deviceInfo->maxOutputChannels; ++a ){
            int b = deviceInfo->maxInputChannels + a;
            cwAsioDeviceInfo->asioChannelInfos[b].channel = a;
            cwAsioDeviceInfo->asioChannelInfos[b].isInput = ASIOFalse;
            cwASIOError asioError = cwASIOGetChannelInfo( &cwAsioDeviceInfo->asioChannelInfos[b] );
            if( asioError != ASE_OK )
            {
                result = paUnanticipatedHostError;
                PA_CWASIO_SET_LAST_ASIO_ERROR( asioError );
                goto error_unload;
            }
        }

        /* unload the driver */
        UnloadAsioDriver();
    }

    return result;

error_unload:
    UnloadAsioDriver();

    if(cwAsioDeviceInfo->asioChannelInfos ){
        PaUtil_GroupFreeMemory( cwAsioHostApi->allocations, cwAsioDeviceInfo->asioChannelInfos );
        cwAsioDeviceInfo->asioChannelInfos = 0;
    }

    return result;
}


static long cwAsioGetNumDevs()
{
    CwAsioDriverInfos cwAsioDriverInfos;
    long numDevs = getDriverNames( &cwAsioDriverInfos, NULL, 32U );
    return numDevs;
}

#if _WINDOWS
    /* we look up IsDebuggerPresent at runtime incase it isn't present (on Win95 for example) */
    typedef BOOL (WINAPI *IsDebuggerPresentPtr)(VOID);
    static IsDebuggerPresentPtr IsDebuggerPresent_ = NULL;
    //static FARPROC IsDebuggerPresent_ = 0; // this is the current way to do it apparently according to davidv
#else
    typedef bool (*IsDebuggerPresentPtr)(void);
    static IsDebuggerPresentPtr IsDebuggerPresent_ = NULL;
#endif

PaError PaCwAsio_Initialize( PaUtilHostApiRepresentation **hostApi, PaHostApiIndex hostApiIndex )
{
    MessageBoxA(NULL, "Bla bla\nBla bla\nBla bla\nBla bla\nBla bla\nBla bla\nBla bla\nBla bla\nBla bla\nBla bla\n", "Debug", MB_OK);
    PaError result = paNoError;
    int i, driverCount;
    PaCwAsioHostApiRepresentation *cwAsioHostApi;
    PaCwAsioDeviceInfo *deviceInfoArray;
    char **names;
    cwAsioHostApi = (PaCwAsioHostApiRepresentation*)PaUtil_AllocateZeroInitializedMemory( sizeof(PaCwAsioHostApiRepresentation) );
    if( !cwAsioHostApi )
    {
        result = paInsufficientMemory;
        goto error;
    }

    /* NOTE: we depend on PaUtil_AllocateZeroInitializedMemory() ensuring that all
       fields are set to zero. especially cwAsioHostApi->allocations */

    /*
        COM will be handled correctly by cwASIO. No need for us to take care about this.
    */
    cwAsioHostApi->allocations = PaUtil_CreateAllocationGroup();
    if( !cwAsioHostApi->allocations )
    {
        result = paInsufficientMemory;
        goto error;
    }

    cwAsioHostApi->systemSpecific = 0;
    cwAsioHostApi->openAsioDeviceIndex = paNoDevice;

    *hostApi = &cwAsioHostApi->inheritedHostApiRep;
    (*hostApi)->info.structVersion = 1;

    (*hostApi)->info.type = paCwASIO;
    (*hostApi)->info.name = "cwASIO";
    (*hostApi)->info.deviceCount = 0;

    #ifdef WINDOWS
        /* use desktop window as system specific ptr */
        cwAsioHostApi->systemSpecific = GetDesktopWindow();
    #endif

    /* driverCount is the number of installed drivers - not necessarily
        the number of installed physical devices. */
    driverCount = cwAsioGetNumDevs();

    if( driverCount > 0 )
    {
        names = GetCwAsioDriverNames( cwAsioHostApi, cwAsioHostApi->allocations, driverCount );
        if( !names )
        {
            result = paInsufficientMemory;
            goto error;
        }


        /* allocate enough space for all drivers, even if some aren't installed */

        (*hostApi)->deviceInfos = (PaDeviceInfo**)PaUtil_GroupAllocateZeroInitializedMemory(
                cwAsioHostApi->allocations, sizeof(PaDeviceInfo*) * driverCount );
        if( !(*hostApi)->deviceInfos )
        {
            result = paInsufficientMemory;
            goto error;
        }

        /* allocate all device info structs in a contiguous block */
        deviceInfoArray = (PaCwAsioDeviceInfo*)PaUtil_GroupAllocateZeroInitializedMemory(
                cwAsioHostApi->allocations, sizeof(PaCwAsioDeviceInfo) * driverCount );
        if( !deviceInfoArray )
        {
            result = paInsufficientMemory;
            goto error;
        }

#if _WINDOWS
        IsDebuggerPresent_ = (IsDebuggerPresentPtr)GetProcAddress( LoadLibraryA( "Kernel32.dll" ), "IsDebuggerPresent" );
#endif

        for( i=0; i < driverCount; ++i )
        {
            PA_DEBUG(("ASIO names[%d]:%s\n",i,names[i]));

            // Since portaudio opens ALL ASIO drivers, and no one else does that,
            // we face fact that some drivers were not meant for it, drivers which act
            // like shells on top of REAL drivers, for instance.
            // so we get duplicate handles, locks and other problems.
            // so lets NOT try to load any such wrappers.
            // The ones i [davidv] know of so far are:

            if (   strcmp (names[i],"ASIO DirectX Full Duplex Driver") == 0
                || strcmp (names[i],"ASIO Multimedia Driver")          == 0
                || strncmp(names[i],"Premiere",8)                      == 0   //"Premiere Elements Windows Sound 1.0"
                || strncmp(names[i],"Adobe",5)                         == 0   //"Adobe Default Windows Sound 1.5"
               )
            {
                PA_DEBUG(("BLACKLISTED!!!\n"));
                continue;
            }


            if( IsDebuggerPresent_ && IsDebuggerPresent_() )
            {
                /* ASIO Digidesign Driver uses PACE copy protection which quits out
                   if a debugger is running. So we don't load it if a debugger is running. */
                if( strcmp(names[i], "ASIO Digidesign Driver") == 0 )
                {
                    PA_DEBUG(("BLACKLISTED!!! ASIO Digidesign Driver would quit the debugger\n"));
                    continue;
                }
            }


            /* Attempt to init device info from the asio driver... */
            {
                PaCwAsioDeviceInfo *cwAsioDeviceInfo = &deviceInfoArray[ (*hostApi)->info.deviceCount ];
                PaDeviceInfo *deviceInfo = &cwAsioDeviceInfo->commonDeviceInfo;

                deviceInfo->structVersion = 2;
                deviceInfo->hostApi = hostApiIndex;

                deviceInfo->name = names[i];

                if( InitPaDeviceInfoFromAsioDriver( cwAsioHostApi, names[i], i, deviceInfo, cwAsioDeviceInfo) == paNoError )
                {
                    (*hostApi)->deviceInfos[ (*hostApi)->info.deviceCount ] = deviceInfo;
                    ++(*hostApi)->info.deviceCount;
                }
                else
                {
                    PA_DEBUG(("Skipping ASIO device:%s\n",names[i]));
                    continue;
                }
            }
        }
    }

    if( (*hostApi)->info.deviceCount > 0 )
    {
        (*hostApi)->info.defaultInputDevice = 0;
        (*hostApi)->info.defaultOutputDevice = 0;
    }
    else
    {
        (*hostApi)->info.defaultInputDevice = paNoDevice;
        (*hostApi)->info.defaultOutputDevice = paNoDevice;
    }


    (*hostApi)->Terminate = Terminate;
    (*hostApi)->OpenStream = OpenStream;
    (*hostApi)->IsFormatSupported = IsFormatSupported;

    PaUtil_InitializeStreamInterface( &cwAsioHostApi->callbackStreamInterface, CloseStream, StartStream,
                                      StopStream, AbortStream, IsStreamStopped, IsStreamActive,
                                      GetStreamTime, GetStreamCpuLoad,
                                      PaUtil_DummyRead, PaUtil_DummyWrite,
                                      PaUtil_DummyGetReadAvailable, PaUtil_DummyGetWriteAvailable );

    PaUtil_InitializeStreamInterface( &cwAsioHostApi->blockingStreamInterface, CloseStream, StartStream,
                                      StopStream, AbortStream, IsStreamStopped, IsStreamActive,
                                      GetStreamTime, PaUtil_DummyGetCpuLoad,
                                      ReadStream, WriteStream, GetStreamReadAvailable, GetStreamWriteAvailable );

    return result;

error:
    if( cwAsioHostApi )
    {
        if( cwAsioHostApi->allocations )
        {
            PaUtil_FreeAllAllocations( cwAsioHostApi->allocations );
            PaUtil_DestroyAllocationGroup( cwAsioHostApi->allocations );
        }

        PaUtil_FreeMemory( cwAsioHostApi );
    }

    return result;
}


static void Terminate( struct PaUtilHostApiRepresentation *hostApi )
{
    PaCwAsioHostApiRepresentation *cwAsioHostApi = (PaCwAsioHostApiRepresentation*)hostApi;

    /*
        IMPLEMENT ME:
            - clean up any resources not handled by the allocation group (need to review if there are any)
    */

    if( cwAsioHostApi->allocations )
    {
        PaUtil_FreeAllAllocations( cwAsioHostApi->allocations );
        PaUtil_DestroyAllocationGroup( cwAsioHostApi->allocations );
    }

    PaUtil_FreeMemory( cwAsioHostApi );
}


static PaError IsFormatSupported( struct PaUtilHostApiRepresentation *hostApi,
                                  const PaStreamParameters *inputParameters,
                                  const PaStreamParameters *outputParameters,
                                  double sampleRate )
{
    PaError result = paNoError;
    PaCwAsioHostApiRepresentation *cwAsioHostApi = (PaCwAsioHostApiRepresentation*)hostApi;
    PaCwAsioDriverInfo *driverInfo = &cwAsioHostApi->openAsioDriverInfo;
    int inputChannelCount, outputChannelCount;
    PaSampleFormat inputSampleFormat, outputSampleFormat;
    PaDeviceIndex asioDeviceIndex;
    cwASIOError asioError;

    if( inputParameters && outputParameters )
    {
        /* full duplex ASIO stream must use the same device for input and output */

        if( inputParameters->device != outputParameters->device )
            return paBadIODeviceCombination;
    }

    if( inputParameters )
    {
        inputChannelCount = inputParameters->channelCount;
        inputSampleFormat = inputParameters->sampleFormat;

        /* all standard sample formats are supported by the buffer adapter,
            this implementation doesn't support any custom sample formats */
        if( inputSampleFormat & paCustomFormat )
            return paSampleFormatNotSupported;

        /* unless alternate device specification is supported, reject the use of
            paUseHostApiSpecificDeviceSpecification */

        if( inputParameters->device == paUseHostApiSpecificDeviceSpecification )
            return paInvalidDevice;

        asioDeviceIndex = inputParameters->device;

        /* validate inputStreamInfo */
        /** @todo do more validation here */
        // if( inputParameters->hostApiSpecificStreamInfo )
        //    return paIncompatibleHostApiSpecificStreamInfo; /* this implementation doesn't use custom stream info */
    }
    else
    {
        inputChannelCount = 0;
    }

    if( outputParameters )
    {
        outputChannelCount = outputParameters->channelCount;
        outputSampleFormat = outputParameters->sampleFormat;

        /* all standard sample formats are supported by the buffer adapter,
            this implementation doesn't support any custom sample formats */
        if( outputSampleFormat & paCustomFormat )
            return paSampleFormatNotSupported;

        /* unless alternate device specification is supported, reject the use of
            paUseHostApiSpecificDeviceSpecification */

        if( outputParameters->device == paUseHostApiSpecificDeviceSpecification )
            return paInvalidDevice;

        asioDeviceIndex = outputParameters->device;

        /* validate outputStreamInfo */
        /** @todo do more validation here */
        // if( outputParameters->hostApiSpecificStreamInfo )
        //    return paIncompatibleHostApiSpecificStreamInfo; /* this implementation doesn't use custom stream info */
    }
    else
    {
        outputChannelCount = 0;
    }



    /* if an ASIO device is open we can only get format information for the currently open device */

    if( cwAsioHostApi->openAsioDeviceIndex != paNoDevice
            && cwAsioHostApi->openAsioDeviceIndex != asioDeviceIndex )
    {
        return paDeviceUnavailable;
    }


    /* NOTE: we load the driver and use its current settings
        rather than the ones in our device info structure which may be stale */

    /* open the device if it's not already open */
    if( cwAsioHostApi->openAsioDeviceIndex == paNoDevice )
    {
        result = LoadAsioDriver( cwAsioHostApi, cwAsioHostApi->inheritedHostApiRep.deviceInfos[ asioDeviceIndex ]->name,
                driverInfo, cwAsioHostApi->systemSpecific );
        if( result != paNoError )
            return result;
    }

    /* check that input device can support inputChannelCount */
    if( inputChannelCount > 0 )
    {
        if( inputChannelCount > driverInfo->inputChannelCount )
        {
            result = paInvalidChannelCount;
            goto done;
        }
    }

    /* check that output device can support outputChannelCount */
    if( outputChannelCount )
    {
        if( outputChannelCount > driverInfo->outputChannelCount )
        {
            result = paInvalidChannelCount;
            goto done;
        }
    }

    /* query for sample rate support */
    asioError = cwASIOCanSampleRate( sampleRate );
    if( asioError == ASE_NoClock || asioError == ASE_NotPresent )
    {
        result = paInvalidSampleRate;
        goto done;
    }

done:
    /* close the device if it wasn't already open */
    if( cwAsioHostApi->openAsioDeviceIndex == paNoDevice )
    {
        UnloadAsioDriver(); /* not sure if we should check for errors here */
    }

    if( result == paNoError )
        return paFormatIsSupported;
    else
        return result;
}



/** A data structure specifically for storing blocking i/o related data. */
typedef struct PaAsioStreamBlockingState
{
    bool stopFlag; /**< Flag indicating that block processing is to be stopped. */

    unsigned long writeBuffersRequested; /**< The number of available output buffers, requested by the #WriteStream() function. */
    unsigned long readFramesRequested;   /**< The number of available input frames, requested by the #ReadStream() function. */

    bool writeBuffersRequestedFlag; /**< Flag to indicate that #WriteStream() has requested more output buffers to be available. */
    bool readFramesRequestedFlag;   /**< Flag to indicate that #ReadStream() requires more input frames to be available. */

    neosmart_event_t writeBuffersReadyEvent; /**< Event to signal that requested output buffers are available. */
    neosmart_event_t readFramesReadyEvent;   /**< Event to signal that requested input frames are available. */

    void *writeRingBufferData; /**< The actual ring buffer memory, used by the output ring buffer. */
    void *readRingBufferData;  /**< The actual ring buffer memory, used by the input ring buffer. */

    PaUtilRingBuffer writeRingBuffer; /**< Frame-aligned blocking i/o ring buffer to store output data (interleaved user format). */
    PaUtilRingBuffer readRingBuffer;  /**< Frame-aligned blocking i/o ring buffer to store input data (interleaved user format). */

    long writeRingBufferInitialFrames; /**< The initial number of silent frames within the output ring buffer. */

    const void **writeStreamBuffer; /**< Temp buffer, used by #WriteStream() for handling non-interleaved data. */
    void **readStreamBuffer; /**< Temp buffer, used by #ReadStream() for handling non-interleaved data. */

    PaUtilBufferProcessor bufferProcessor; /**< Buffer processor, used to handle the blocking i/o ring buffers. */

    int outputUnderflowFlag; /**< Flag to signal an output underflow from within the callback function. */
    int inputOverflowFlag; /**< Flag to signal an input overflow from within the callback function. */
}
PaAsioStreamBlockingState;



/* PaAsioStream - a stream data structure specifically for this implementation */

typedef struct PaAsioStream
{
    PaUtilStreamRepresentation streamRepresentation;
    PaUtilCpuLoadMeasurer cpuLoadMeasurer;
    PaUtilBufferProcessor bufferProcessor;

    PaCwAsioHostApiRepresentation *cwAsioHostApi;
    unsigned long framesPerHostCallback;

    /* ASIO driver info  - these may not be needed for the life of the stream,
        but store them here until we work out how format conversion is going
        to work. */

    struct cwASIOBufferInfo *asioBufferInfos;
    struct cwASIOChannelInfo *asioChannelInfos;
    long asioInputLatencyFrames, asioOutputLatencyFrames; // actual latencies returned by asio

    long inputChannelCount, outputChannelCount;
    bool postOutput;

    void **bufferPtrs; /* this is carved up for inputBufferPtrs and outputBufferPtrs */
    void **inputBufferPtrs[2];
    void **outputBufferPtrs[2];

    PaCwAsioBufferConverter *inputBufferConverter;
    long inputShift;
    PaCwAsioBufferConverter *outputBufferConverter;
    long outputShift;

    volatile bool stopProcessing;
    int stopPlayoutCount;
    neosmart_event_t completedBuffersPlayedEvent;

    bool streamFinishedCallbackCalled;
    int isStopped;
    volatile int isActive;
    volatile bool zeroOutput; /* all future calls to the callback will output silence */

    volatile long reenterCount;
    volatile long reenterError;

    PaStreamCallbackFlags callbackFlags;

    PaAsioStreamBlockingState *blockingState; /**< Blocking i/o data struct, or NULL when using callback interface. */
}
PaAsioStream;

static PaAsioStream *theAsioStream = 0; /* due to ASIO sdk limitations there can be only one stream */


static void ZeroOutputBuffers( PaAsioStream *stream, long index )
{
    int i;

    for( i=0; i < stream->outputChannelCount; ++i )
    {
        void *buffer = stream->asioBufferInfos[ i + stream->inputChannelCount ].buffers[index];

        int bytesPerSample = BytesPerAsioSample( stream->asioChannelInfos[ i + stream->inputChannelCount ].type );

        memset( buffer, 0, stream->framesPerHostCallback * bytesPerSample );
    }
}


/* return the next power of two >= x.
   Returns the input parameter if it is already a power of two.
   http://stackoverflow.com/questions/364985/algorithm-for-finding-the-smallest-power-of-two-thats-greater-or-equal-to-a-giv
*/
static unsigned long NextPowerOfTwo( unsigned long x )
{
    --x;
    x |= x >> 1;
    x |= x >> 2;
    x |= x >> 4;
    x |= x >> 8;
    x |= x >> 16;
    /* If you needed to deal with numbers > 2^32 the following would be needed.
       For latencies, we don't deal with values this large.
     x |= x >> 16;
    */

    return x + 1;
}


static unsigned long SelectHostBufferSizeForUnspecifiedUserFramesPerBuffer(
        unsigned long targetBufferingLatencyFrames, PaCwAsioDriverInfo *driverInfo )
{
    /* Choose a host buffer size based only on targetBufferingLatencyFrames and the
       device's supported buffer sizes. Always returns a valid value.
    */

    unsigned long result;

    if( targetBufferingLatencyFrames <= (unsigned long)driverInfo->bufferMinSize )
    {
        result = driverInfo->bufferMinSize;
    }
    else if( targetBufferingLatencyFrames >= (unsigned long)driverInfo->bufferMaxSize )
    {
        result = driverInfo->bufferMaxSize;
    }
    else
    {
        if( driverInfo->bufferGranularity == 0 ) /* single fixed host buffer size */
        {
            /* The documentation states that bufferGranularity should be zero
               when bufferMinSize, bufferMaxSize and bufferPreferredSize are the
               same. We assume that is the case.
            */

            result = driverInfo->bufferPreferredSize;
        }
        else if( driverInfo->bufferGranularity == -1 ) /* power-of-two */
        {
            /* We assume bufferMinSize and bufferMaxSize are powers of two. */

            result = NextPowerOfTwo( targetBufferingLatencyFrames );

            if( result < (unsigned long)driverInfo->bufferMinSize )
                result = driverInfo->bufferMinSize;

            if( result > (unsigned long)driverInfo->bufferMaxSize )
                result = driverInfo->bufferMaxSize;
        }
        else /* modulo bufferGranularity */
        {
            /* round up to the next multiple of granularity */
            unsigned long n = (targetBufferingLatencyFrames + driverInfo->bufferGranularity - 1)
                    / driverInfo->bufferGranularity;

            result = n * driverInfo->bufferGranularity;

            if( result < (unsigned long)driverInfo->bufferMinSize )
                result = driverInfo->bufferMinSize;

            if( result > (unsigned long)driverInfo->bufferMaxSize )
                result = driverInfo->bufferMaxSize;
        }
    }

    return result;
}


static unsigned long SelectHostBufferSizeForSpecifiedUserFramesPerBuffer(
        unsigned long targetBufferingLatencyFrames, unsigned long userFramesPerBuffer,
        PaCwAsioDriverInfo *driverInfo )
{
    /* Select a host buffer size conforming to targetBufferingLatencyFrames
       and the device's supported buffer sizes.
       The return value will always be a multiple of userFramesPerBuffer.
       If a valid buffer size can not be found the function returns 0.

       The current implementation uses a simple iterative search for clarity.
       Feel free to suggest a closed form solution.
    */
    unsigned long result = 0;

    assert( userFramesPerBuffer != 0 );

    if( driverInfo->bufferGranularity == 0 ) /* single fixed host buffer size */
    {
        /* The documentation states that bufferGranularity should be zero
           when bufferMinSize, bufferMaxSize and bufferPreferredSize are the
           same. We assume that is the case.
        */

        if( (driverInfo->bufferPreferredSize % userFramesPerBuffer) == 0 )
            result = driverInfo->bufferPreferredSize;
    }
    else if( driverInfo->bufferGranularity == -1 ) /* power-of-two */
    {
        /* We assume bufferMinSize and bufferMaxSize are powers of two. */

        /* Search all powers of two in the range [bufferMinSize,bufferMaxSize]
           for multiples of userFramesPerBuffer. We prefer the first multiple
           that is equal or greater than targetBufferingLatencyFrames, or
           failing that, the largest multiple less than
           targetBufferingLatencyFrames.
        */
        unsigned long x = (unsigned long)driverInfo->bufferMinSize;
        do {
            if( (x % userFramesPerBuffer) == 0 )
            {
                /* any multiple of userFramesPerBuffer is acceptable */
                result = x;
                if( result >= targetBufferingLatencyFrames )
                    break; /* stop. a value >= to targetBufferingLatencyFrames is ideal. */
            }

            x *= 2;
        } while( x <= (unsigned long)driverInfo->bufferMaxSize );
    }
    else /* modulo granularity */
    {
        /* We assume bufferMinSize is a multiple of bufferGranularity. */

        /* Search all multiples of bufferGranularity in the range
           [bufferMinSize,bufferMaxSize] for multiples of userFramesPerBuffer.
           We prefer the first multiple that is equal or greater than
           targetBufferingLatencyFrames, or failing that, the largest multiple
           less than targetBufferingLatencyFrames.
        */
        unsigned long x = (unsigned long)driverInfo->bufferMinSize;
        do {
            if( (x % userFramesPerBuffer) == 0 )
            {
                /* any multiple of userFramesPerBuffer is acceptable */
                result = x;
                if( result >= targetBufferingLatencyFrames )
                    break; /* stop. a value >= to targetBufferingLatencyFrames is ideal. */
            }

            x += driverInfo->bufferGranularity;
        } while( x <= (unsigned long)driverInfo->bufferMaxSize );
    }

    return result;
}


static unsigned long SelectHostBufferSize(
        unsigned long targetBufferingLatencyFrames,
        unsigned long userFramesPerBuffer, PaCwAsioDriverInfo *driverInfo )
{
    unsigned long result = 0;

    /* We select a host buffer size based on the following requirements
       (in priority order):

        1. The host buffer size must be permissible according to the ASIO
           driverInfo buffer size constraints (min, max, granularity or
           powers-of-two).

        2. If the user specifies a non-zero framesPerBuffer parameter
           (userFramesPerBuffer here) the host buffer should be a multiple of
           this (subject to the constraints in (1) above).

           [NOTE: Where no permissible host buffer size is a multiple of
           userFramesPerBuffer, we choose a value as if userFramesPerBuffer were
           zero (i.e. we ignore it). This strategy is open for review ~ perhaps
           there are still "more optimal" buffer sizes related to
           userFramesPerBuffer that we could use.]

        3. The host buffer size should be greater than or equal to
           targetBufferingLatencyFrames, subject to (1) and (2) above. Where it
           is not possible to select a host buffer size equal or greater than
           targetBufferingLatencyFrames, the highest buffer size conforming to
           (1) and (2) should be chosen.
    */

    if( userFramesPerBuffer != 0 )
    {
        /* userFramesPerBuffer is specified, try to find a buffer size that's
           a multiple of it */
        result = SelectHostBufferSizeForSpecifiedUserFramesPerBuffer(
                targetBufferingLatencyFrames, userFramesPerBuffer, driverInfo );
    }

    if( result == 0 )
    {
        /* either userFramesPerBuffer was not specified, or we couldn't find a
           host buffer size that is a multiple of it. Select a host buffer size
           according to targetBufferingLatencyFrames and the ASIO driverInfo
           buffer size constraints.
         */
        result = SelectHostBufferSizeForUnspecifiedUserFramesPerBuffer(
                targetBufferingLatencyFrames, driverInfo );
    }

    return result;
}


/* returns channelSelectors if present */

static PaError ValidateAsioSpecificStreamInfo(
        const PaStreamParameters *streamParameters,
        const PaCwAsioStreamInfo *streamInfo,
        int deviceChannelCount,
        int **channelSelectors )
{
    if( streamInfo )
    {
        if( streamInfo->size != sizeof( PaCwAsioStreamInfo )
                || streamInfo->version != 1 )
        {
            return paIncompatibleHostApiSpecificStreamInfo;
        }

        if( streamInfo->flags & paCwAsioUseChannelSelectors )
        {
            *channelSelectors = streamInfo->channelSelectors;

            if( !(*channelSelectors) )
                return paIncompatibleHostApiSpecificStreamInfo;

            for( int i=0; i < streamParameters->channelCount; ++i ){
                if( (*channelSelectors)[i] < 0
                    || (*channelSelectors)[i] >= deviceChannelCount ){
                    return paInvalidChannelCount;
                }
            }
        }
    }

    return paNoError;
}


static bool IsUsingExternalClockSource()
{
    bool result = false;
    cwASIOError asioError;
    struct cwASIOClockSource clocks[32];
    long numSources=32;

    /* davidv: listing ASIO Clock sources. there is an ongoing investigation by
       me about whether or not to call cwASIOSetSampleRate if an external Clock is
       used. A few drivers expected different things here */

    asioError = cwASIOGetClockSources(clocks, &numSources);
    if( asioError != ASE_OK ){
        PA_DEBUG(("ERROR: cwASIOGetClockSources: %s\n", PaAsio_GetAsioErrorText(asioError) ));
    }else{
        PA_DEBUG(("INFO cwASIOGetClockSources listing %d clocks\n", numSources ));
        for (int i=0;i<numSources;++i){
            PA_DEBUG(("cwASIOClockSource%d %s current:%d\n", i, clocks[i].name, clocks[i].isCurrentSource ));

            if (clocks[i].isCurrentSource)
                result = true;
        }
    }

    return result;
}


static PaError ValidateAndSetSampleRate( double sampleRate )
{
    PaError result = paNoError;
    cwASIOError asioError;

    // check that the device supports the requested sample rate

    asioError = cwASIOCanSampleRate( sampleRate );
    PA_DEBUG(("cwASIOCanSampleRate(%f):%d\n", sampleRate, asioError ));

    if( asioError != ASE_OK )
    {
        result = paInvalidSampleRate;
        PA_DEBUG(("ERROR: cwASIOCanSampleRate: %s\n", PaAsio_GetAsioErrorText(asioError) ));
        goto error;
    }

    // retrieve the current sample rate, we only change to the requested
    // sample rate if the device is not already in that rate.

    cwASIOSampleRate oldRate;
    asioError = cwASIOGetSampleRate(&oldRate);
    if( asioError != ASE_OK )
    {
        result = paInvalidSampleRate;
        PA_DEBUG(("ERROR: cwASIOGetSampleRate: %s\n", PaAsio_GetAsioErrorText(asioError) ));
        goto error;
    }
    PA_DEBUG(("cwASIOGetSampleRate:%f\n",oldRate));

    if (oldRate != sampleRate){
        /* Set sample rate */

        PA_DEBUG(("before cwASIOSetSampleRate(%f)\n",sampleRate));

        /*
            If you have problems with some drivers when externally clocked,
            try switching on the following line and commenting out the one after it.
            See IsUsingExternalClockSource() for more info.
        */
        //if( IsUsingExternalClockSource() ){
        if( false ){
            asioError = cwASIOSetSampleRate( 0 );
        }else{
            asioError = cwASIOSetSampleRate( sampleRate );
        }
        if( asioError != ASE_OK )
        {
            result = paInvalidSampleRate;
            PA_DEBUG(("ERROR: cwASIOSetSampleRate: %s\n", PaAsio_GetAsioErrorText(asioError) ));
            goto error;
        }
        PA_DEBUG(("after cwASIOSetSampleRate(%f)\n",sampleRate));
    }
    else
    {
        PA_DEBUG(("No Need to change SR\n"));
    }

error:
    return result;
}


/* see pa_hostapi.h for a list of validity guarantees made about OpenStream  parameters */

static PaError OpenStream( struct PaUtilHostApiRepresentation *hostApi,
                           PaStream** s,
                           const PaStreamParameters *inputParameters,
                           const PaStreamParameters *outputParameters,
                           double sampleRate,
                           unsigned long framesPerBuffer,
                           PaStreamFlags streamFlags,
                           PaStreamCallback *streamCallback,
                           void *userData )
{
    PaError result = paNoError;
    PaCwAsioHostApiRepresentation *cwAsioHostApi = (PaCwAsioHostApiRepresentation*)hostApi;
    PaAsioStream *stream = 0;
    PaCwAsioStreamInfo *inputStreamInfo, *outputStreamInfo;
    unsigned long framesPerHostBuffer;
    int inputChannelCount, outputChannelCount;
    PaSampleFormat inputSampleFormat, outputSampleFormat;
    PaSampleFormat hostInputSampleFormat, hostOutputSampleFormat;
    unsigned long suggestedInputLatencyFrames;
    unsigned long suggestedOutputLatencyFrames;
    PaDeviceIndex asioDeviceIndex;
    cwASIOError asioError;
    int asioIsInitialized = 0;
    int asioBuffersCreated = 0;
    int completedBuffersPlayedEventInited = 0;
    int i;
    PaCwAsioDriverInfo *driverInfo;
    int *inputChannelSelectors = 0;
    int *outputChannelSelectors = 0;

    /* Are we using blocking i/o interface? */
    bool usingBlockingIo = ( !streamCallback ) ? true : false;
    /* Blocking i/o stuff */
    long lBlockingBufferSize     = 0; /* Desired ring buffer size in samples. */
    long lBlockingBufferSizePow2 = 0; /* Power-of-2 rounded ring buffer size. */
    long lBytesPerFrame          = 0; /* Number of bytes per input/output frame. */
    int blockingWriteBuffersReadyEventInitialized = 0; /* Event init flag. */
    int blockingReadFramesReadyEventInitialized   = 0; /* Event init flag. */

    bool callbackBufferProcessorInited = false;
    bool blockingBufferProcessorInited = false;

    /* unless we move to using lower level ASIO calls, we can only have
        one device open at a time */
    if( cwAsioHostApi->openAsioDeviceIndex != paNoDevice )
    {
        PA_DEBUG(("OpenStream paDeviceUnavailable\n"));
        return paDeviceUnavailable;
    }

    assert( theAsioStream == 0 );

    if( inputParameters && outputParameters )
    {
        /* full duplex ASIO stream must use the same device for input and output */

        if( inputParameters->device != outputParameters->device )
        {
            PA_DEBUG(("OpenStream paBadIODeviceCombination\n"));
            return paBadIODeviceCombination;
        }
    }

    if( inputParameters )
    {
        inputChannelCount = inputParameters->channelCount;
        inputSampleFormat = inputParameters->sampleFormat;
        suggestedInputLatencyFrames = (unsigned long)((inputParameters->suggestedLatency * sampleRate)+0.5f);

        /* unless alternate device specification is supported, reject the use of
            paUseHostApiSpecificDeviceSpecification */
        if( inputParameters->device == paUseHostApiSpecificDeviceSpecification )
            return paInvalidDevice;

        asioDeviceIndex = inputParameters->device;

        PaCwAsioDeviceInfo *cwAsioDeviceInfo = (PaCwAsioDeviceInfo*)hostApi->deviceInfos[asioDeviceIndex];

        /* validate hostApiSpecificStreamInfo */
        inputStreamInfo = (PaCwAsioStreamInfo*)inputParameters->hostApiSpecificStreamInfo;
        result = ValidateAsioSpecificStreamInfo( inputParameters, inputStreamInfo,
            cwAsioDeviceInfo->commonDeviceInfo.maxInputChannels,
            &inputChannelSelectors
        );
        if( result != paNoError ) return result;
    }
    else
    {
        inputChannelCount = 0;
        inputSampleFormat = 0;
        suggestedInputLatencyFrames = 0;
    }

    if( outputParameters )
    {
        outputChannelCount = outputParameters->channelCount;
        outputSampleFormat = outputParameters->sampleFormat;
        suggestedOutputLatencyFrames = (unsigned long)((outputParameters->suggestedLatency * sampleRate)+0.5f);

        /* unless alternate device specification is supported, reject the use of
            paUseHostApiSpecificDeviceSpecification */
        if( outputParameters->device == paUseHostApiSpecificDeviceSpecification )
            return paInvalidDevice;

        asioDeviceIndex = outputParameters->device;

        PaCwAsioDeviceInfo * cwAsioDeviceInfo = (PaCwAsioDeviceInfo*)hostApi->deviceInfos[asioDeviceIndex];

        /* validate hostApiSpecificStreamInfo */
        outputStreamInfo = (PaCwAsioStreamInfo*)outputParameters->hostApiSpecificStreamInfo;
        result = ValidateAsioSpecificStreamInfo( outputParameters, outputStreamInfo,
            cwAsioDeviceInfo->commonDeviceInfo.maxOutputChannels,
            &outputChannelSelectors
        );
        if( result != paNoError ) return result;
    }
    else
    {
        outputChannelCount = 0;
        outputSampleFormat = 0;
        suggestedOutputLatencyFrames = 0;
    }

    driverInfo = &cwAsioHostApi->openAsioDriverInfo;

    /* NOTE: we load the driver and use its current settings
        rather than the ones in our device info structure which may be stale */

    result = LoadAsioDriver( cwAsioHostApi, cwAsioHostApi->inheritedHostApiRep.deviceInfos[ asioDeviceIndex ]->name,
            driverInfo, cwAsioHostApi->systemSpecific );
    if( result == paNoError )
        asioIsInitialized = 1;
    else{
        PA_DEBUG(("OpenStream ERROR1 - LoadAsioDriver returned %d\n", result));
        goto error;
    }

    /* check that input device can support inputChannelCount */
    if( inputChannelCount > 0 )
    {
        if( inputChannelCount > driverInfo->inputChannelCount )
        {
            result = paInvalidChannelCount;
            PA_DEBUG(("OpenStream ERROR2\n"));
            goto error;
        }
    }

    /* check that output device can support outputChannelCount */
    if( outputChannelCount )
    {
        if( outputChannelCount > driverInfo->outputChannelCount )
        {
            result = paInvalidChannelCount;
            PA_DEBUG(("OpenStream ERROR3\n"));
            goto error;
        }
    }

    result = ValidateAndSetSampleRate( sampleRate );
    if( result != paNoError )
        goto error;

    /*
        IMPLEMENT ME:
            - if a full duplex stream is requested, check that the combination
                of input and output parameters is supported
    */

    /* validate platform specific flags */
    if( (streamFlags & paPlatformSpecificFlags) != 0 ){
        PA_DEBUG(("OpenStream invalid flags!!\n"));
        return paInvalidFlag; /* unexpected platform specific flag */
    }


    stream = (PaAsioStream*)PaUtil_AllocateZeroInitializedMemory( sizeof(PaAsioStream) );
    if( !stream )
    {
        result = paInsufficientMemory;
        PA_DEBUG(("OpenStream ERROR5\n"));
        goto error;
    }
    stream->blockingState = NULL; /* Blocking i/o not initialized, yet. */


    stream->completedBuffersPlayedEvent = NspeCreateEvent( true, false );
    if( stream->completedBuffersPlayedEvent == NULL )
    {
        result = paUnanticipatedHostError;
        PA_CWASIO_SET_LAST_SYSTEM_ERROR( getLastError() );
        PA_DEBUG(("OpenStream ERROR6\n"));
        goto error;
    }
    completedBuffersPlayedEventInited = 1;


    stream->asioBufferInfos = 0; /* for deallocation in error */
    stream->asioChannelInfos = 0; /* for deallocation in error */
    stream->bufferPtrs = 0; /* for deallocation in error */

    /* Using blocking i/o interface... */
    if( usingBlockingIo )
    {
        /* Blocking i/o is implemented by running callback mode, using a special blocking i/o callback. */
        streamCallback = BlockingIoPaCallback; /* Setup PA to use the ASIO blocking i/o callback. */
        userData       = &theAsioStream;       /* The callback user data will be the PA ASIO stream. */
        PaUtil_InitializeStreamRepresentation( &stream->streamRepresentation,
                                               &cwAsioHostApi->blockingStreamInterface, streamCallback, userData );
    }
    else /* Using callback interface... */
    {
        PaUtil_InitializeStreamRepresentation( &stream->streamRepresentation,
                                               &cwAsioHostApi->callbackStreamInterface, streamCallback, userData );
    }


    PaUtil_InitializeCpuLoadMeasurer( &stream->cpuLoadMeasurer, sampleRate );


    stream->asioBufferInfos = (struct cwASIOBufferInfo*)PaUtil_AllocateZeroInitializedMemory(
            sizeof(struct cwASIOBufferInfo) * (inputChannelCount + outputChannelCount) );
    if( !stream->asioBufferInfos )
    {
        result = paInsufficientMemory;
        PA_DEBUG(("OpenStream ERROR7\n"));
        goto error;
    }


    for( i=0; i < inputChannelCount; ++i )
    {
        struct cwASIOBufferInfo *info = &stream->asioBufferInfos[i];

        info->isInput = ASIOTrue;

        if( inputChannelSelectors ){
            // inputChannelSelectors values have already been validated in
            // ValidateAsioSpecificStreamInfo() above
            info->channelNum = inputChannelSelectors[i];
        }else{
            info->channelNum = i;
        }

        info->buffers[0] = info->buffers[1] = 0;
    }

    for( i=0; i < outputChannelCount; ++i ){
        struct cwASIOBufferInfo *info = &stream->asioBufferInfos[inputChannelCount+i];

        info->isInput = ASIOFalse;

        if( outputChannelSelectors ){
            // outputChannelSelectors values have already been validated in
            // ValidateAsioSpecificStreamInfo() above
            info->channelNum = outputChannelSelectors[i];
        }else{
            info->channelNum = i;
        }

        info->buffers[0] = info->buffers[1] = 0;
    }


    /* Using blocking i/o interface... */
    if( usingBlockingIo )
    {
/** @todo REVIEW selection of host buffer size for blocking i/o */

        framesPerHostBuffer = SelectHostBufferSize( 0, framesPerBuffer, driverInfo );

    }
    else /* Using callback interface... */
    {
        /* Select the host buffer size based on user framesPerBuffer and the
           maximum of suggestedInputLatencyFrames and
           suggestedOutputLatencyFrames.

           We should subtract any fixed known driver latency from
           suggestedLatencyFrames before computing the host buffer size.
           However, the ASIO API doesn't provide a method for determining fixed
           latencies independent of the host buffer size. cwASIOGetLatencies()
           only returns latencies after the buffer size has been configured, so
           we can't reliably use it to determine fixed latencies here.

           We could set the preferred buffer size and then subtract it from
           the values returned from cwASIOGetLatencies, but this would not be 100%
           reliable, so we don't do it.
        */

        unsigned long targetBufferingLatencyFrames =
                (( suggestedInputLatencyFrames > suggestedOutputLatencyFrames )
                ? suggestedInputLatencyFrames
                : suggestedOutputLatencyFrames);

        framesPerHostBuffer = SelectHostBufferSize( targetBufferingLatencyFrames,
                framesPerBuffer, driverInfo );
    }


    PA_DEBUG(("PaAsioOpenStream: framesPerHostBuffer :%d\n",  framesPerHostBuffer));

    asioError = cwASIOCreateBuffers( stream->asioBufferInfos,
            inputChannelCount+outputChannelCount,
            framesPerHostBuffer, &asioCallbacks_ );

    if( asioError != ASE_OK
            && framesPerHostBuffer != (unsigned long)driverInfo->bufferPreferredSize )
    {
        PA_DEBUG(("ERROR: cwASIOCreateBuffers: %s\n", PaAsio_GetAsioErrorText(asioError) ));
        /*
            Some buggy drivers (like the Hoontech DSP24) give incorrect
            [min, preferred, max] values They should work with the preferred size
            value, thus if Pa_ASIO_CreateBuffers fails with the hostBufferSize
            computed in SelectHostBufferSize, we try again with the preferred size.
        */

        framesPerHostBuffer = driverInfo->bufferPreferredSize;

        PA_DEBUG(("PaAsioOpenStream: CORRECTED framesPerHostBuffer :%d\n",  framesPerHostBuffer));

        cwASIOError asioError2 = cwASIOCreateBuffers( stream->asioBufferInfos,
                inputChannelCount+outputChannelCount,
                 framesPerHostBuffer, &asioCallbacks_ );
        if( asioError2 == ASE_OK )
            asioError = ASE_OK;
    }

    if( asioError != ASE_OK )
    {
        result = paUnanticipatedHostError;
        PA_CWASIO_SET_LAST_ASIO_ERROR( asioError );
        PA_DEBUG(("OpenStream ERROR9\n"));
        goto error;
    }

    asioBuffersCreated = 1;

    stream->asioChannelInfos = (struct cwASIOChannelInfo*)PaUtil_AllocateZeroInitializedMemory(
            sizeof(struct cwASIOChannelInfo) * (inputChannelCount + outputChannelCount) );
    if( !stream->asioChannelInfos )
    {
        result = paInsufficientMemory;
        PA_DEBUG(("OpenStream ERROR10\n"));
        goto error;
    }

    for( i=0; i < inputChannelCount + outputChannelCount; ++i )
    {
        stream->asioChannelInfos[i].channel = stream->asioBufferInfos[i].channelNum;
        stream->asioChannelInfos[i].isInput = stream->asioBufferInfos[i].isInput;
        asioError = cwASIOGetChannelInfo( &stream->asioChannelInfos[i] );
        if( asioError != ASE_OK )
        {
            result = paUnanticipatedHostError;
            PA_CWASIO_SET_LAST_ASIO_ERROR( asioError );
            PA_DEBUG(("OpenStream ERROR11\n"));
            goto error;
        }
    }

    stream->bufferPtrs = (void**)PaUtil_AllocateZeroInitializedMemory(
            2 * sizeof(void*) * (inputChannelCount + outputChannelCount) );
    if( !stream->bufferPtrs )
    {
        result = paInsufficientMemory;
        PA_DEBUG(("OpenStream ERROR12\n"));
        goto error;
    }

    if( inputChannelCount > 0 )
    {
        stream->inputBufferPtrs[0] = stream-> bufferPtrs;
        stream->inputBufferPtrs[1] = &stream->bufferPtrs[inputChannelCount];

        for( i=0; i<inputChannelCount; ++i )
        {
            stream->inputBufferPtrs[0][i] = stream->asioBufferInfos[i].buffers[0];
            stream->inputBufferPtrs[1][i] = stream->asioBufferInfos[i].buffers[1];
        }
    }
    else
    {
        stream->inputBufferPtrs[0] = 0;
        stream->inputBufferPtrs[1] = 0;
    }

    if( outputChannelCount > 0 )
    {
        stream->outputBufferPtrs[0] = &stream->bufferPtrs[inputChannelCount*2];
        stream->outputBufferPtrs[1] = &stream->bufferPtrs[inputChannelCount*2 + outputChannelCount];

        for( i=0; i<outputChannelCount; ++i )
        {
            stream->outputBufferPtrs[0][i] = stream->asioBufferInfos[inputChannelCount+i].buffers[0];
            stream->outputBufferPtrs[1][i] = stream->asioBufferInfos[inputChannelCount+i].buffers[1];
        }
    }
    else
    {
        stream->outputBufferPtrs[0] = 0;
        stream->outputBufferPtrs[1] = 0;
    }

    if( inputChannelCount > 0 )
    {
        /* FIXME: assume all channels use the same type for now

            see: "ASIO devices with multiple sample formats are unsupported"
            http://www.portaudio.com/trac/ticket/106
        */
        cwASIOSampleType inputType = stream->asioChannelInfos[0].type;

        PA_DEBUG(("ASIO Input  type:%d",inputType));
        CwAsioSampleTypeLOG(inputType);
        hostInputSampleFormat = AsioSampleTypeToPaNativeSampleFormat( inputType );

        SelectAsioToPaConverter( inputType, &stream->inputBufferConverter, &stream->inputShift );
    }
    else
    {
        hostInputSampleFormat = 0;
        stream->inputBufferConverter = 0;
    }

    if( outputChannelCount > 0 )
    {
        /* FIXME: assume all channels use the same type for now

            see: "ASIO devices with multiple sample formats are unsupported"
            http://www.portaudio.com/trac/ticket/106
        */
        cwASIOSampleType outputType = stream->asioChannelInfos[inputChannelCount].type;

        PA_DEBUG(("ASIO Output type:%d",outputType));
        CwAsioSampleTypeLOG(outputType);
        hostOutputSampleFormat = AsioSampleTypeToPaNativeSampleFormat( outputType );

        SelectPaToAsioConverter( outputType, &stream->outputBufferConverter, &stream->outputShift );
    }
    else
    {
        hostOutputSampleFormat = 0;
        stream->outputBufferConverter = 0;
    }

    /* Values returned by cwASIOGetLatencies() include the latency introduced by
       the ASIO double buffer. */
    cwASIOGetLatencies( &stream->asioInputLatencyFrames, &stream->asioOutputLatencyFrames );


    /* Using blocking i/o interface... */
    if( usingBlockingIo )
    {
        /* Allocate the blocking i/o input ring buffer memory. */
        stream->blockingState = (PaAsioStreamBlockingState*)PaUtil_AllocateZeroInitializedMemory( sizeof(PaAsioStreamBlockingState) );
        if( !stream->blockingState )
        {
            result = paInsufficientMemory;
            PA_DEBUG(("ERROR! Blocking i/o interface struct allocation failed in OpenStream()\n"));
            goto error;
        }

        /* Initialize blocking i/o interface struct. */
        stream->blockingState->readFramesReadyEvent   = NULL; /* Uninitialized, yet. */
        stream->blockingState->writeBuffersReadyEvent = NULL; /* Uninitialized, yet. */
        stream->blockingState->readRingBufferData     = NULL; /* Uninitialized, yet. */
        stream->blockingState->writeRingBufferData    = NULL; /* Uninitialized, yet. */
        stream->blockingState->readStreamBuffer       = NULL; /* Uninitialized, yet. */
        stream->blockingState->writeStreamBuffer      = NULL; /* Uninitialized, yet. */
        stream->blockingState->stopFlag               = true; /* Not started, yet. */


        /* If the user buffer is unspecified */
        if( framesPerBuffer == paFramesPerBufferUnspecified )
        {
            /* Make the user buffer the same size as the host buffer. */
            framesPerBuffer = framesPerHostBuffer;
        }


        /* Initialize callback buffer processor. */
        result = PaUtil_InitializeBufferProcessor( &stream->bufferProcessor               ,
                                                    inputChannelCount                     ,
                                                    inputSampleFormat & ~paNonInterleaved , /* Ring buffer. */
                                                    (hostInputSampleFormat | paNonInterleaved), /* Host format. */
                                                    outputChannelCount                    ,
                                                    outputSampleFormat & ~paNonInterleaved, /* Ring buffer. */
                                                    (hostOutputSampleFormat | paNonInterleaved), /* Host format. */
                                                    sampleRate                            ,
                                                    streamFlags                           ,
                                                    framesPerBuffer                       , /* Frames per ring buffer block. */
                                                    framesPerHostBuffer                   , /* Frames per asio buffer. */
                                                    paUtilFixedHostBufferSize             ,
                                                    streamCallback                        ,
                                                    userData                               );
        if( result != paNoError ){
            PA_DEBUG(("OpenStream ERROR13\n"));
            goto error;
        }
        callbackBufferProcessorInited = true;

        /* Initialize the blocking i/o buffer processor. */
        result = PaUtil_InitializeBufferProcessor(&stream->blockingState->bufferProcessor,
                                                   inputChannelCount                     ,
                                                   inputSampleFormat                     , /* User format. */
                                                   inputSampleFormat & ~paNonInterleaved , /* Ring buffer. */
                                                   outputChannelCount                    ,
                                                   outputSampleFormat                    , /* User format. */
                                                   outputSampleFormat & ~paNonInterleaved, /* Ring buffer. */
                                                   sampleRate                            ,
                                                   paClipOff | paDitherOff               , /* Don't use dither nor clipping. */
                                                   framesPerBuffer                       , /* Frames per user buffer. */
                                                   framesPerBuffer                       , /* Frames per ring buffer block. */
                                                   paUtilBoundedHostBufferSize           ,
                                                   NULL, NULL                            );/* No callback! */
        if( result != paNoError ){
            PA_DEBUG(("ERROR! Blocking i/o buffer processor initialization failed in OpenStream()\n"));
            goto error;
        }
        blockingBufferProcessorInited = true;

        /* If input is requested. */
        if( inputChannelCount )
        {
            /* Create the callback sync-event. */
            stream->blockingState->readFramesReadyEvent = NspeCreateEvent( false, false );
            if( stream->blockingState->readFramesReadyEvent == NULL )
            {
                result = paUnanticipatedHostError;
                PA_CWASIO_SET_LAST_SYSTEM_ERROR( getLastError() );
                PA_DEBUG(("ERROR! Blocking i/o \"read frames ready\" event creation failed in OpenStream()\n"));
                goto error;
            }
            blockingReadFramesReadyEventInitialized = 1;


            /* Create pointer buffer to access non-interleaved data in ReadStream() */
            stream->blockingState->readStreamBuffer = (void**)PaUtil_AllocateZeroInitializedMemory( sizeof(void*) * inputChannelCount );
            if( !stream->blockingState->readStreamBuffer )
            {
                result = paInsufficientMemory;
                PA_DEBUG(("ERROR! Blocking i/o read stream buffer allocation failed in OpenStream()\n"));
                goto error;
            }

            /* The ring buffer should store as many data blocks as needed
               to achieve the requested latency. Whereas it must be large
               enough to store at least two complete data blocks.

               1) Determine the amount of latency to be added to the
                  preferred ASIO latency.
               2) Make sure we have at lest one additional latency frame.
               3) Divide the number of frames by the desired block size to
                  get the number (rounded up to pure integer) of blocks to
                  be stored in the buffer.
               4) Add one additional block for block processing and convert
                  to samples frames.
               5) Get the next larger (or equal) power-of-two buffer size.
             */
            lBlockingBufferSize = suggestedInputLatencyFrames - stream->asioInputLatencyFrames;
            lBlockingBufferSize = (lBlockingBufferSize > 0) ? lBlockingBufferSize : 1;
            lBlockingBufferSize = (lBlockingBufferSize + framesPerBuffer - 1) / framesPerBuffer;
            lBlockingBufferSize = (lBlockingBufferSize + 1) * framesPerBuffer;

            /* Get the next larger or equal power-of-two buffersize. */
            lBlockingBufferSizePow2 = 1;
            while( lBlockingBufferSize > (lBlockingBufferSizePow2<<=1) );
            lBlockingBufferSize = lBlockingBufferSizePow2;

            /* Compute total input latency in seconds */
            stream->streamRepresentation.streamInfo.inputLatency =
                (double)( PaUtil_GetBufferProcessorInputLatencyFrames(&stream->bufferProcessor               )
                        + PaUtil_GetBufferProcessorInputLatencyFrames(&stream->blockingState->bufferProcessor)
                        + (lBlockingBufferSize / framesPerBuffer - 1) * framesPerBuffer
                        + stream->asioInputLatencyFrames )
                / sampleRate;

            /* The code below prints the ASIO latency which doesn't include
               the buffer processor latency nor the blocking i/o latency. It
               reports the added latency separately.
            */
            PA_DEBUG(("PaAsio : ASIO InputLatency = %ld (%ld ms),\n         added buffProc:%ld (%ld ms),\n         added blocking:%ld (%ld ms)\n",
                stream->asioInputLatencyFrames,
                (long)( stream->asioInputLatencyFrames * (1000.0 / sampleRate) ),
                PaUtil_GetBufferProcessorInputLatencyFrames(&stream->bufferProcessor),
                (long)( PaUtil_GetBufferProcessorInputLatencyFrames(&stream->bufferProcessor) * (1000.0 / sampleRate) ),
                PaUtil_GetBufferProcessorInputLatencyFrames(&stream->blockingState->bufferProcessor) + (lBlockingBufferSize / framesPerBuffer - 1) * framesPerBuffer,
                (long)( (PaUtil_GetBufferProcessorInputLatencyFrames(&stream->blockingState->bufferProcessor) + (lBlockingBufferSize / framesPerBuffer - 1) * framesPerBuffer) * (1000.0 / sampleRate) )
                ));

            /* Determine the size of ring buffer in bytes. */
            lBytesPerFrame = inputChannelCount * Pa_GetSampleSize(inputSampleFormat );

            /* Allocate the blocking i/o input ring buffer memory. */
            stream->blockingState->readRingBufferData = (void*)PaUtil_AllocateZeroInitializedMemory( lBlockingBufferSize * lBytesPerFrame );
            if( !stream->blockingState->readRingBufferData )
            {
                result = paInsufficientMemory;
                PA_DEBUG(("ERROR! Blocking i/o input ring buffer allocation failed in OpenStream()\n"));
                goto error;
            }

            /* Initialize the input ring buffer struct. */
            PaUtil_InitializeRingBuffer( &stream->blockingState->readRingBuffer    ,
                                          lBytesPerFrame                           ,
                                          lBlockingBufferSize                      ,
                                          stream->blockingState->readRingBufferData );
        }

        /* If output is requested. */
        if( outputChannelCount )
        {
            stream->blockingState->writeBuffersReadyEvent = NspeCreateEvent( false, false );
            if( stream->blockingState->writeBuffersReadyEvent == NULL )
            {
                result = paUnanticipatedHostError;
                PA_CWASIO_SET_LAST_SYSTEM_ERROR( getLastError() );
                PA_DEBUG(("ERROR! Blocking i/o \"write buffers ready\" event creation failed in OpenStream()\n"));
                goto error;
            }
            blockingWriteBuffersReadyEventInitialized = 1;

            /* Create pointer buffer to access non-interleaved data in WriteStream() */
            stream->blockingState->writeStreamBuffer = (const void**)PaUtil_AllocateZeroInitializedMemory( sizeof(const void*) * outputChannelCount );
            if( !stream->blockingState->writeStreamBuffer )
            {
                result = paInsufficientMemory;
                PA_DEBUG(("ERROR! Blocking i/o write stream buffer allocation failed in OpenStream()\n"));
                goto error;
            }

            /* The ring buffer should store as many data blocks as needed
               to achieve the requested latency. Whereas it must be large
               enough to store at least two complete data blocks.

               1) Determine the amount of latency to be added to the
                  preferred ASIO latency.
               2) Make sure we have at lest one additional latency frame.
               3) Divide the number of frames by the desired block size to
                  get the number (rounded up to pure integer) of blocks to
                  be stored in the buffer.
               4) Add one additional block for block processing and convert
                  to samples frames.
               5) Get the next larger (or equal) power-of-two buffer size.
             */
            lBlockingBufferSize = suggestedOutputLatencyFrames - stream->asioOutputLatencyFrames;
            lBlockingBufferSize = (lBlockingBufferSize > 0) ? lBlockingBufferSize : 1;
            lBlockingBufferSize = (lBlockingBufferSize + framesPerBuffer - 1) / framesPerBuffer;
            lBlockingBufferSize = (lBlockingBufferSize + 1) * framesPerBuffer;

            /* The buffer size (without the additional block) corresponds
               to the initial number of silent samples in the output ring
               buffer. */
            stream->blockingState->writeRingBufferInitialFrames = lBlockingBufferSize - framesPerBuffer;

            /* Get the next larger or equal power-of-two buffersize. */
            lBlockingBufferSizePow2 = 1;
            while( lBlockingBufferSize > (lBlockingBufferSizePow2<<=1) );
            lBlockingBufferSize = lBlockingBufferSizePow2;

            /* Compute total output latency in seconds */
            stream->streamRepresentation.streamInfo.outputLatency =
                (double)( PaUtil_GetBufferProcessorOutputLatencyFrames(&stream->bufferProcessor)
                        + PaUtil_GetBufferProcessorOutputLatencyFrames(&stream->blockingState->bufferProcessor)
                        + (lBlockingBufferSize / framesPerBuffer - 1) * framesPerBuffer
                        + stream->asioOutputLatencyFrames )
                / sampleRate;

            /* The code below prints the ASIO latency which doesn't include
               the buffer processor latency nor the blocking i/o latency. It
               reports the added latency separately.
            */
            PA_DEBUG(("PaAsio : ASIO OutputLatency = %ld (%ld ms),\n         added buffProc:%ld (%ld ms),\n         added blocking:%ld (%ld ms)\n",
                stream->asioOutputLatencyFrames,
                (long)( stream->asioOutputLatencyFrames * (1000.0 / sampleRate) ),
                PaUtil_GetBufferProcessorOutputLatencyFrames(&stream->bufferProcessor),
                (long)( PaUtil_GetBufferProcessorOutputLatencyFrames(&stream->bufferProcessor) * (1000.0 / sampleRate) ),
                PaUtil_GetBufferProcessorOutputLatencyFrames(&stream->blockingState->bufferProcessor) + (lBlockingBufferSize / framesPerBuffer - 1) * framesPerBuffer,
                (long)( (PaUtil_GetBufferProcessorOutputLatencyFrames(&stream->blockingState->bufferProcessor) + (lBlockingBufferSize / framesPerBuffer - 1) * framesPerBuffer) * (1000.0 / sampleRate) )
                ));

            /* Determine the size of ring buffer in bytes. */
            lBytesPerFrame = outputChannelCount * Pa_GetSampleSize(outputSampleFormat);

            /* Allocate the blocking i/o output ring buffer memory. */
            stream->blockingState->writeRingBufferData = (void*)PaUtil_AllocateZeroInitializedMemory( lBlockingBufferSize * lBytesPerFrame );
            if( !stream->blockingState->writeRingBufferData )
            {
                result = paInsufficientMemory;
                PA_DEBUG(("ERROR! Blocking i/o output ring buffer allocation failed in OpenStream()\n"));
                goto error;
            }

            /* Initialize the output ring buffer struct. */
            PaUtil_InitializeRingBuffer( &stream->blockingState->writeRingBuffer    ,
                                          lBytesPerFrame                            ,
                                          lBlockingBufferSize                       ,
                                          stream->blockingState->writeRingBufferData );
        }

        stream->streamRepresentation.streamInfo.sampleRate = sampleRate;


    }
    else /* Using callback interface... */
    {
        result =  PaUtil_InitializeBufferProcessor( &stream->bufferProcessor,
                        inputChannelCount, inputSampleFormat, (hostInputSampleFormat | paNonInterleaved),
                        outputChannelCount, outputSampleFormat, (hostOutputSampleFormat | paNonInterleaved),
                        sampleRate, streamFlags, framesPerBuffer,
                        framesPerHostBuffer, paUtilFixedHostBufferSize,
                        streamCallback, userData );
        if( result != paNoError ){
            PA_DEBUG(("OpenStream ERROR13\n"));
            goto error;
        }
        callbackBufferProcessorInited = true;

        stream->streamRepresentation.streamInfo.inputLatency =
                (double)( PaUtil_GetBufferProcessorInputLatencyFrames(&stream->bufferProcessor)
                    + stream->asioInputLatencyFrames) / sampleRate;   // seconds
        stream->streamRepresentation.streamInfo.outputLatency =
                (double)( PaUtil_GetBufferProcessorOutputLatencyFrames(&stream->bufferProcessor)
                    + stream->asioOutputLatencyFrames) / sampleRate; // seconds
        stream->streamRepresentation.streamInfo.sampleRate = sampleRate;

        // the code below prints the ASIO latency which doesn't include the
        // buffer processor latency. it reports the added latency separately
        PA_DEBUG(("PaAsio : ASIO InputLatency = %ld (%ld ms), added buffProc:%ld (%ld ms)\n",
                stream->asioInputLatencyFrames,
                (long)((stream->asioInputLatencyFrames*1000)/ sampleRate),
                PaUtil_GetBufferProcessorInputLatencyFrames(&stream->bufferProcessor),
                (long)((PaUtil_GetBufferProcessorInputLatencyFrames(&stream->bufferProcessor)*1000)/ sampleRate)
                ));

        PA_DEBUG(("PaAsio : ASIO OuputLatency = %ld (%ld ms), added buffProc:%ld (%ld ms)\n",
                stream->asioOutputLatencyFrames,
                (long)((stream->asioOutputLatencyFrames*1000)/ sampleRate),
                PaUtil_GetBufferProcessorOutputLatencyFrames(&stream->bufferProcessor),
                (long)((PaUtil_GetBufferProcessorOutputLatencyFrames(&stream->bufferProcessor)*1000)/ sampleRate)
                ));
    }

    stream->cwAsioHostApi = cwAsioHostApi;
    stream->framesPerHostCallback = framesPerHostBuffer;

    stream->inputChannelCount = inputChannelCount;
    stream->outputChannelCount = outputChannelCount;
    stream->postOutput = driverInfo->postOutput;
    stream->isStopped = 1;
    stream->isActive = 0;

    cwAsioHostApi->openAsioDeviceIndex = asioDeviceIndex;

    theAsioStream = stream;
    *s = (PaStream*)stream;

    return result;

error:
    PA_DEBUG(("goto errored\n"));
    if( stream )
    {
        if( stream->blockingState )
        {
            if( blockingBufferProcessorInited )
                PaUtil_TerminateBufferProcessor( &stream->blockingState->bufferProcessor );

            if( stream->blockingState->writeRingBufferData )
                PaUtil_FreeMemory( stream->blockingState->writeRingBufferData );
            if( stream->blockingState->writeStreamBuffer )
                PaUtil_FreeMemory( stream->blockingState->writeStreamBuffer );
            if( blockingWriteBuffersReadyEventInitialized )
                NspeDestroyEvent( stream->blockingState->writeBuffersReadyEvent );

            if( stream->blockingState->readRingBufferData )
                PaUtil_FreeMemory( stream->blockingState->readRingBufferData );
            if( stream->blockingState->readStreamBuffer )
                PaUtil_FreeMemory( stream->blockingState->readStreamBuffer );
            if( blockingReadFramesReadyEventInitialized )
                NspeDestroyEvent( stream->blockingState->readFramesReadyEvent );

            PaUtil_FreeMemory( stream->blockingState );
        }

        if( callbackBufferProcessorInited )
            PaUtil_TerminateBufferProcessor( &stream->bufferProcessor );

        if( completedBuffersPlayedEventInited )
            NspeDestroyEvent( stream->completedBuffersPlayedEvent );

        if( stream->asioBufferInfos )
            PaUtil_FreeMemory( stream->asioBufferInfos );

        if( stream->asioChannelInfos )
            PaUtil_FreeMemory( stream->asioChannelInfos );

        if( stream->bufferPtrs )
            PaUtil_FreeMemory( stream->bufferPtrs );

        PaUtil_FreeMemory( stream );
    }

    if( asioBuffersCreated )
        cwASIODisposeBuffers();

    if( asioIsInitialized )
    {
        UnloadAsioDriver();
    }
    return result;
}


/*
    When CloseStream() is called, the multi-api layer ensures that
    the stream has already been stopped or aborted.
*/
static PaError CloseStream( PaStream* s )
{
    PaError result = paNoError;
    PaAsioStream *stream = (PaAsioStream*)s;

    /*
        IMPLEMENT ME:
            - additional stream closing + cleanup
    */

    PaUtil_TerminateBufferProcessor( &stream->bufferProcessor );
    PaUtil_TerminateStreamRepresentation( &stream->streamRepresentation );

    stream->cwAsioHostApi->openAsioDeviceIndex = paNoDevice;

    NspeDestroyEvent( stream->completedBuffersPlayedEvent );

    /* Using blocking i/o interface... */
    if( stream->blockingState )
    {
        PaUtil_TerminateBufferProcessor( &stream->blockingState->bufferProcessor );

        if( stream->inputChannelCount ) {
            PaUtil_FreeMemory( stream->blockingState->readRingBufferData );
            PaUtil_FreeMemory( stream->blockingState->readStreamBuffer  );
            NspeDestroyEvent( stream->blockingState->readFramesReadyEvent );
        }
        if( stream->outputChannelCount ) {
            PaUtil_FreeMemory( stream->blockingState->writeRingBufferData );
            PaUtil_FreeMemory( stream->blockingState->writeStreamBuffer );
            NspeDestroyEvent( stream->blockingState->writeBuffersReadyEvent );
        }

        PaUtil_FreeMemory( stream->blockingState );
    }

    PaUtil_FreeMemory( stream->asioBufferInfos );
    PaUtil_FreeMemory( stream->asioChannelInfos );
    PaUtil_FreeMemory( stream->bufferPtrs );
    PaUtil_FreeMemory( stream );

    cwASIODisposeBuffers();
    UnloadAsioDriver();

    theAsioStream = 0;

    return result;
}


static void bufferSwitch(long index, cwASIOBool directProcess)
{
//TAKEN FROM THE ASIO SDK

    // the actual processing callback.
    // Beware that this is normally in a separate thread, hence be sure that
    // you take care about thread synchronization. This is omitted here for
    // simplicity.

    // as this is a "back door" into the bufferSwitchTimeInfo a timeInfo needs
    // to be created though it will only set the timeInfo.samplePosition and
    // timeInfo.systemTime fields and the according flags

    struct cwASIOTime  timeInfo;
    memset( &timeInfo, 0, sizeof (timeInfo) );

    // get the time stamp of the buffer, not necessary if no
    // synchronization to other media is required
    if( cwASIOGetSamplePosition(&timeInfo.timeInfo.samplePosition, &timeInfo.timeInfo.systemTime) == ASE_OK)
            timeInfo.timeInfo.flags = kSystemTimeValid | kSamplePositionValid;

    // Call the real callback
    bufferSwitchTimeInfo( &timeInfo, index, directProcess );
}


// conversion from 64 bit ASIOSample/cwASIOTimeStamp to double float
#if NATIVE_INT64
    #define ASIO64toDouble(a)  (a)
#else
    const double twoRaisedTo32 = 4294967296.;
    #define ASIO64toDouble(a)  ((a).lo + (a).hi * twoRaisedTo32)
#endif

static struct cwASIOTime *bufferSwitchTimeInfo( struct cwASIOTime *timeInfo, long index, cwASIOBool directProcess )
{
    // the actual processing callback.
    // Beware that this is normally in a separate thread, hence be sure that
    // you take care about thread synchronization.


    /* The SDK says the following about the directProcess flag:
        suggests to the host whether it should immediately start processing
        (directProcess == ASIOTrue), or whether its process should be deferred
        because the call comes from a very low level (for instance, a high level
        priority interrupt), and direct processing would cause timing instabilities for
        the rest of the system. If in doubt, directProcess should be set to ASIOFalse.

        We just ignore directProcess. This could cause incompatibilities with
        drivers which really don't want the audio processing to occur in this
        callback, but none have been identified yet.
    */

    (void) directProcess; /* suppress unused parameter warning */

#if 0
    // store the timeInfo for later use
    asioDriverInfo.tInfo = *timeInfo;

    // get the time stamp of the buffer, not necessary if no
    // synchronization to other media is required

    if (timeInfo->timeInfo.flags & kSystemTimeValid)
            asioDriverInfo.nanoSeconds = ASIO64toDouble(timeInfo->timeInfo.systemTime);
    else
            asioDriverInfo.nanoSeconds = 0;

    if (timeInfo->timeInfo.flags & kSamplePositionValid)
            asioDriverInfo.samples = ASIO64toDouble(timeInfo->timeInfo.samplePosition);
    else
            asioDriverInfo.samples = 0;

    if (timeInfo->timeCode.flags & kTcValid)
            asioDriverInfo.tcSamples = ASIO64toDouble(timeInfo->timeCode.timeCodeSamples);
    else
            asioDriverInfo.tcSamples = 0;

    // get the system reference time
    asioDriverInfo.sysRefTime = get_sys_reference_time();
#endif

#if 0
    // a few debug messages for the Windows device driver developer
    // tells you the time when driver got its interrupt and the delay until the app receives
    // the event notification.
    static double last_samples = 0;
    char tmp[128];
    sprintf (tmp, "diff: %d / %d ms / %d ms / %d samples                 \n", asioDriverInfo.sysRefTime - (long)(asioDriverInfo.nanoSeconds / 1000000.0), asioDriverInfo.sysRefTime, (long)(asioDriverInfo.nanoSeconds / 1000000.0), (long)(asioDriverInfo.samples - last_samples));
    OutputDebugString (tmp);
    last_samples = asioDriverInfo.samples;
#endif


    if( !theAsioStream )
        return 0L;

    // protect against reentrancy
    if( PaCwAsio_AtomicIncrement(&theAsioStream->reenterCount) )
    {
        theAsioStream->reenterError++;
        //DBUG(("bufferSwitchTimeInfo : reentrancy detection = %d\n", asioDriverInfo.reenterError));
        return 0L;
    }

    int buffersDone = 0;

    do
    {
        if( buffersDone > 0 )
        {
            // this is a reentered buffer, we missed processing it on time
            // set the input overflow and output underflow flags as appropriate

            if( theAsioStream->inputChannelCount > 0 )
                theAsioStream->callbackFlags |= paInputOverflow;

            if( theAsioStream->outputChannelCount > 0 )
                theAsioStream->callbackFlags |= paOutputUnderflow;
        }
        else
        {
            if( theAsioStream->zeroOutput )
            {
                ZeroOutputBuffers( theAsioStream, index );

                // Finally if the driver supports the cwASIOOutputReady() optimization,
                // do it here, all data are in place
                if( theAsioStream->postOutput )
                    cwASIOOutputReady();

                if( theAsioStream->stopProcessing )
                {
                    if( theAsioStream->stopPlayoutCount < 2 )
                    {
                        ++theAsioStream->stopPlayoutCount;
                        if( theAsioStream->stopPlayoutCount == 2 )
                        {
                            theAsioStream->isActive = 0;
                            if( theAsioStream->streamRepresentation.streamFinishedCallback != 0 )
                                theAsioStream->streamRepresentation.streamFinishedCallback( theAsioStream->streamRepresentation.userData );
                            theAsioStream->streamFinishedCallbackCalled = true;
                            NspeSetEvent( theAsioStream->completedBuffersPlayedEvent );
                        }
                    }
                }
            }
            else
            {

#if 0
/*
    see: "ASIO callback underflow/overflow buffer slip detection doesn't work"
    http://www.portaudio.com/trac/ticket/110
*/

// test code to try to detect slip conditions... these may work on some systems
// but neither of them work on the RME Digi96

// check that sample delta matches buffer size (otherwise we must have skipped
// a buffer.
static double last_samples = -512;
double samples;
//if( timeInfo->timeCode.flags & kTcValid )
//    samples = ASIO64toDouble(timeInfo->timeCode.timeCodeSamples);
//else
    samples = ASIO64toDouble(timeInfo->timeInfo.samplePosition);
int delta = samples - last_samples;
//printf( "%d\n", delta);
last_samples = samples;

if( delta > theAsioStream->framesPerHostCallback )
{
    if( theAsioStream->inputChannelCount > 0 )
        theAsioStream->callbackFlags |= paInputOverflow;

    if( theAsioStream->outputChannelCount > 0 )
        theAsioStream->callbackFlags |= paOutputUnderflow;
}

// check that the buffer index is not the previous index (which would indicate
// that a buffer was skipped.
static int previousIndex = 1;
if( index == previousIndex )
{
    if( theAsioStream->inputChannelCount > 0 )
        theAsioStream->callbackFlags |= paInputOverflow;

    if( theAsioStream->outputChannelCount > 0 )
        theAsioStream->callbackFlags |= paOutputUnderflow;
}
previousIndex = index;
#endif

                int i;

                PaUtil_BeginCpuLoadMeasurement( &theAsioStream->cpuLoadMeasurer );

                PaStreamCallbackTimeInfo paTimeInfo;

                // asio systemTime is supposed to be measured according to the same
                // clock as timeGetTime
                paTimeInfo.currentTime = (ASIO64toDouble( timeInfo->timeInfo.systemTime ) * .000000001);

                /* patch from Paul Boege */
                paTimeInfo.inputBufferAdcTime = paTimeInfo.currentTime -
                    ((double)theAsioStream->asioInputLatencyFrames/theAsioStream->streamRepresentation.streamInfo.sampleRate);

                paTimeInfo.outputBufferDacTime = paTimeInfo.currentTime +
                    ((double)theAsioStream->asioOutputLatencyFrames/theAsioStream->streamRepresentation.streamInfo.sampleRate);

                /* old version is buggy because the buffer processor also adds in its latency to the time parameters
                paTimeInfo.inputBufferAdcTime = paTimeInfo.currentTime - theAsioStream->streamRepresentation.streamInfo.inputLatency;
                paTimeInfo.outputBufferDacTime = paTimeInfo.currentTime + theAsioStream->streamRepresentation.streamInfo.outputLatency;
                */

/* Disabled! Stopping and re-starting the stream causes an input overflow / output underflow. S.Fischer */
#if 0
// detect underflows by checking inter-callback time > 2 buffer period
static double previousTime = -1;
if( previousTime > 0 ){

    double delta = paTimeInfo.currentTime - previousTime;

    if( delta >= 2. * (theAsioStream->framesPerHostCallback / theAsioStream->streamRepresentation.streamInfo.sampleRate) ){
        if( theAsioStream->inputChannelCount > 0 )
            theAsioStream->callbackFlags |= paInputOverflow;

        if( theAsioStream->outputChannelCount > 0 )
            theAsioStream->callbackFlags |= paOutputUnderflow;
    }
}
previousTime = paTimeInfo.currentTime;
#endif

                // note that the above input and output times do not need to be
                // adjusted for the latency of the buffer processor -- the buffer
                // processor handles that.

                if( theAsioStream->inputBufferConverter )
                {
                    for( i=0; i<theAsioStream->inputChannelCount; i++ )
                    {
                        theAsioStream->inputBufferConverter( theAsioStream->inputBufferPtrs[index][i],
                                theAsioStream->inputShift, theAsioStream->framesPerHostCallback );
                    }
                }

                PaUtil_BeginBufferProcessing( &theAsioStream->bufferProcessor, &paTimeInfo, theAsioStream->callbackFlags );

                /* reset status flags once they've been passed to the callback */
                theAsioStream->callbackFlags = 0;

                PaUtil_SetInputFrameCount( &theAsioStream->bufferProcessor, 0 /* default to host buffer size */ );
                for( i=0; i<theAsioStream->inputChannelCount; ++i )
                    PaUtil_SetNonInterleavedInputChannel( &theAsioStream->bufferProcessor, i, theAsioStream->inputBufferPtrs[index][i] );

                PaUtil_SetOutputFrameCount( &theAsioStream->bufferProcessor, 0 /* default to host buffer size */ );
                for( i=0; i<theAsioStream->outputChannelCount; ++i )
                    PaUtil_SetNonInterleavedOutputChannel( &theAsioStream->bufferProcessor, i, theAsioStream->outputBufferPtrs[index][i] );

                int callbackResult;
                if( theAsioStream->stopProcessing )
                    callbackResult = paComplete;
                else
                    callbackResult = paContinue;
                unsigned long framesProcessed = PaUtil_EndBufferProcessing( &theAsioStream->bufferProcessor, &callbackResult );

                if( theAsioStream->outputBufferConverter )
                {
                    for( i=0; i<theAsioStream->outputChannelCount; i++ )
                    {
                        theAsioStream->outputBufferConverter( theAsioStream->outputBufferPtrs[index][i],
                                theAsioStream->outputShift, theAsioStream->framesPerHostCallback );
                    }
                }

                PaUtil_EndCpuLoadMeasurement( &theAsioStream->cpuLoadMeasurer, framesProcessed );

                // Finally if the driver supports the cwASIOOutputReady() optimization,
                // do it here, all data are in place
                if( theAsioStream->postOutput )
                    cwASIOOutputReady();

                if( callbackResult == paContinue )
                {
                    /* nothing special to do */
                }
                else if( callbackResult == paAbort )
                {
                    /* finish playback immediately  */
                    theAsioStream->isActive = 0;
                    if( theAsioStream->streamRepresentation.streamFinishedCallback != 0 )
                        theAsioStream->streamRepresentation.streamFinishedCallback( theAsioStream->streamRepresentation.userData );
                    theAsioStream->streamFinishedCallbackCalled = true;
                    NspeSetEvent( theAsioStream->completedBuffersPlayedEvent );
                    theAsioStream->zeroOutput = true;
                }
                else /* paComplete or other non-zero value indicating complete */
                {
                    /* Finish playback once currently queued audio has completed. */
                    theAsioStream->stopProcessing = true;

                    if( PaUtil_IsBufferProcessorOutputEmpty( &theAsioStream->bufferProcessor ) )
                    {
                        theAsioStream->zeroOutput = true;
                        theAsioStream->stopPlayoutCount = 0;
                    }
                }
            }
        }

        ++buffersDone;
    }while( PaCwAsio_AtomicDecrement(&theAsioStream->reenterCount) >= 0 );

    return 0L;
}


static void sampleRateChanged(cwASIOSampleRate sRate)
{
    // TAKEN FROM THE ASIO SDK
    // do whatever you need to do if the sample rate changed
    // usually this only happens during external sync.
    // Audio processing is not stopped by the driver, actual sample rate
    // might not have even changed, maybe only the sample rate status of an
    // AES/EBU or S/PDIF digital input at the audio device.
    // You might have to update time/sample related conversion routines, etc.

    (void) sRate; /* unused parameter */
    PA_DEBUG( ("sampleRateChanged : %d \n", sRate));
}

static long asioMessages(long selector, long value, void* message, double* opt)
{
// TAKEN FROM THE ASIO SDK
    // currently the parameters "value", "message" and "opt" are not used.
    long ret = 0;

    (void) message; /* unused parameters */
    (void) opt;

    PA_DEBUG( ("asioMessages : %d , %d \n", selector, value));

    switch(selector)
    {
        case kAsioSelectorSupported:
            if(value == kAsioResetRequest
            || value == kAsioEngineVersion
            || value == kAsioResyncRequest
            || value == kAsioLatenciesChanged
            // the following three were added for ASIO 2.0, you don't necessarily have to support them
            || value == kAsioSupportsTimeInfo
            || value == kAsioSupportsTimeCode
            || value == kAsioSupportsInputMonitor)
                    ret = 1L;
            break;

        case kAsioBufferSizeChange:
            //printf("kAsioBufferSizeChange \n");
            break;

        case kAsioResetRequest:
            // defer the task and perform the reset of the driver during the next "safe" situation
            // You cannot reset the driver right now, as this code is called from the driver.
            // Reset the driver is done by completely destruct is. I.e. ASIOStop(), cwASIODisposeBuffers(), Destruction
            // Afterwards you initialize the driver again.

            /*FIXME: commented the next line out

                see: "PA/ASIO ignores some driver notifications it probably shouldn't"
                http://www.portaudio.com/trac/ticket/108
            */
            //asioDriverInfo.stopped;  // In this sample the processing will just stop
            ret = 1L;
            break;

        case kAsioResyncRequest:
            // This informs the application, that the driver encountered some non fatal data loss.
            // It is used for synchronization purposes of different media.
            // Added mainly to work around the Win16Mutex problems in Windows 95/98 with the
            // Windows Multimedia system, which could loose data because the Mutex was hold too long
            // by another thread.
            // However a driver can issue it in other situations, too.
            ret = 1L;
            break;

        case kAsioLatenciesChanged:
            // This will inform the host application that the drivers were latencies changed.
            // Beware, it this does not mean that the buffer sizes have changed!
            // You might need to update internal delay data.
            ret = 1L;
            //printf("kAsioLatenciesChanged \n");
            break;

        case kAsioEngineVersion:
            // return the supported ASIO version of the host application
            // If a host applications does not implement this selector, ASIO 1.0 is assumed
            // by the driver
            ret = 2L;
            break;

        case kAsioSupportsTimeInfo:
            // informs the driver whether the asioCallbacks.bufferSwitchTimeInfo() callback
            // is supported.
            // For compatibility with ASIO 1.0 drivers the host application should always support
            // the "old" bufferSwitch method, too.
            ret = 1;
            break;

        case kAsioSupportsTimeCode:
            // informs the driver whether application is interested in time code info.
            // If an application does not need to know about time code, the driver has less work
            // to do.
            ret = 0;
            break;
    }
    return ret;
}


static PaError StartStream( PaStream *s )
{
    PaError result = paNoError;
    PaAsioStream *stream = (PaAsioStream*)s;
    PaAsioStreamBlockingState *blockingState = stream->blockingState;
    cwASIOError asioError;

    if( stream->outputChannelCount > 0 )
    {
        ZeroOutputBuffers( stream, 0 );
        ZeroOutputBuffers( stream, 1 );
    }

    PaUtil_ResetBufferProcessor( &stream->bufferProcessor );
    stream->stopProcessing = false;
    stream->zeroOutput = false;

    /* Reentrancy counter initialisation */
    stream->reenterCount = -1;
    stream->reenterError = 0;

    stream->callbackFlags = 0;

    if(NspeResetEvent( stream->completedBuffersPlayedEvent ) != 0 )
    {
        result = paUnanticipatedHostError;
        PA_CWASIO_SET_LAST_SYSTEM_ERROR( getLastError() );
    }


    /* Using blocking i/o interface... */
    if( blockingState )
    {
        /* Reset blocking i/o buffer processor. */
        PaUtil_ResetBufferProcessor( &blockingState->bufferProcessor );

        /* If we're about to process some input data. */
        if( stream->inputChannelCount )
        {
            /* Reset callback-ReadStream sync event. */
            if(NspeResetEvent( blockingState->readFramesReadyEvent ) != 0 )
            {
                result = paUnanticipatedHostError;
                PA_CWASIO_SET_LAST_SYSTEM_ERROR( getLastError() );
            }

            /* Flush blocking i/o ring buffer. */
            PaUtil_FlushRingBuffer( &blockingState->readRingBuffer );
            (*blockingState->bufferProcessor.inputZeroer)( blockingState->readRingBuffer.buffer, 1, blockingState->bufferProcessor.inputChannelCount * blockingState->readRingBuffer.bufferSize );
        }

        /* If we're about to process some output data. */
        if( stream->outputChannelCount )
        {
            /* Reset callback-WriteStream sync event. */
            if(NspeResetEvent( blockingState->writeBuffersReadyEvent ) != 0 )
            {
                result = paUnanticipatedHostError;
                PA_CWASIO_SET_LAST_SYSTEM_ERROR( getLastError() );
            }

            /* Flush blocking i/o ring buffer. */
            PaUtil_FlushRingBuffer( &blockingState->writeRingBuffer );
            (*blockingState->bufferProcessor.outputZeroer)( blockingState->writeRingBuffer.buffer, 1, blockingState->bufferProcessor.outputChannelCount * blockingState->writeRingBuffer.bufferSize );

            /* Initialize the output ring buffer to "silence". */
            PaUtil_AdvanceRingBufferWriteIndex( &blockingState->writeRingBuffer, blockingState->writeRingBufferInitialFrames );
        }

        /* Clear requested frames / buffers count. */
        blockingState->writeBuffersRequested     = 0;
        blockingState->readFramesRequested       = 0;
        blockingState->writeBuffersRequestedFlag = false;
        blockingState->readFramesRequestedFlag   = false;
        blockingState->outputUnderflowFlag       = false;
        blockingState->inputOverflowFlag         = false;
        blockingState->stopFlag                  = false;
    }


    if( result == paNoError )
    {
        assert( theAsioStream == stream ); /* theAsioStream should be set correctly in OpenStream */

        /* initialize these variables before the callback has a chance to be invoked */
        stream->isStopped = 0;
        stream->isActive = 1;
        stream->streamFinishedCallbackCalled = false;

        asioError = cwASIOStart();
        if( asioError != ASE_OK )
        {
            stream->isStopped = 1;
            stream->isActive = 0;

            result = paUnanticipatedHostError;
            PA_CWASIO_SET_LAST_ASIO_ERROR( asioError );
        }
    }

    return result;
}

static void EnsureCallbackHasCompleted( PaAsioStream *stream )
{
    // make sure that the callback is not still in-flight after ASIOStop()
    // returns. This has been observed to happen on the Hoontech DSP24 for
    // example.
    int count = 2000;  // only wait for 2 seconds, rather than hanging.
    while( stream->reenterCount != -1 && count > 0 )
    {
        PaCwAsio_SleepMilliseconds(1U);
        --count;
    }
}

static PaError StopStream( PaStream *s )
{
    PaError result = paNoError;
    PaAsioStream *stream = (PaAsioStream*)s;
    PaAsioStreamBlockingState *blockingState = stream->blockingState;
    cwASIOError asioError;

    if( stream->isActive )
    {
        /* If blocking i/o output is in use */
        if( blockingState && stream->outputChannelCount )
        {
            /* Request the whole output buffer to be available. */
            blockingState->writeBuffersRequested = blockingState->writeRingBuffer.bufferSize;
            /* Signalize that additional buffers are need. */
            blockingState->writeBuffersRequestedFlag = true;
            /* Set flag to indicate the playback is to be stopped. */
            blockingState->stopFlag = true;

            /* Wait until requested number of buffers has been freed. Time
               out after twice the blocking i/o output buffer could have
               been consumed. */
            DWORD timeout = (DWORD)( 2 * blockingState->writeRingBuffer.bufferSize * 1000
                                       / stream->streamRepresentation.streamInfo.sampleRate );
            DWORD waitResult = NspeWaitForEvent( blockingState->writeBuffersReadyEvent, timeout );

            /* If something seriously went wrong... */
            if( waitResult == WAIT_FAILED )
            {
                PA_DEBUG(("WaitForEvent() failed in StopStream()\n"));
                result = paUnanticipatedHostError;
                PA_CWASIO_SET_LAST_SYSTEM_ERROR( getLastError() );
            }
            else if( waitResult == WAIT_TIMEOUT )
            {
                PA_DEBUG(("WaitForEvent() timed out in StopStream()\n"));
                result = paTimedOut;
            }
        }

        stream->stopProcessing = true;

        /* wait for the stream to finish playing out enqueued buffers.
            timeout after four times the stream latency.

            @todo should use a better time out value - if the user buffer
            length is longer than the asio buffer size then that should
            be taken into account.
        */
        if(NspeWaitForEvent( stream->completedBuffersPlayedEvent,
                (DWORD)(stream->streamRepresentation.streamInfo.outputLatency * 1000. * 4.) )
                    == WAIT_TIMEOUT )
        {
            PA_DEBUG(("WaitForEvent() timed out in StopStream()\n" ));
        }
    }

    asioError = cwASIOStop();
    if( asioError == ASE_OK )
    {
        EnsureCallbackHasCompleted( stream );
    }
    else
    {
        result = paUnanticipatedHostError;
        PA_CWASIO_SET_LAST_ASIO_ERROR( asioError );
    }

    stream->isStopped = 1;
    stream->isActive = 0;

    if( !stream->streamFinishedCallbackCalled )
    {
        if( stream->streamRepresentation.streamFinishedCallback != 0 )
            stream->streamRepresentation.streamFinishedCallback( stream->streamRepresentation.userData );
    }

    return result;
}

static PaError AbortStream( PaStream *s )
{
    PaError result = paNoError;
    PaAsioStream *stream = (PaAsioStream*)s;
    cwASIOError asioError;

    stream->zeroOutput = true;

    asioError = cwASIOStop();
    if( asioError == ASE_OK )
    {
        EnsureCallbackHasCompleted( stream );
    }
    else
    {
        result = paUnanticipatedHostError;
        PA_CWASIO_SET_LAST_ASIO_ERROR( asioError );
    }

    stream->isStopped = 1;
    stream->isActive = 0;

    if( !stream->streamFinishedCallbackCalled )
    {
        if( stream->streamRepresentation.streamFinishedCallback != 0 )
            stream->streamRepresentation.streamFinishedCallback( stream->streamRepresentation.userData );
    }

    return result;
}


static PaError IsStreamStopped( PaStream *s )
{
    PaAsioStream *stream = (PaAsioStream*)s;

    return stream->isStopped;
}


static PaError IsStreamActive( PaStream *s )
{
    PaAsioStream *stream = (PaAsioStream*)s;

    return stream->isActive;
}


static PaTime GetStreamTime( PaStream *s )
{
    (void) s; /* unused parameter */

#if WINDOWS
    return (double)timeGetTime() * .001;
#else
    struct timespec tp{ 0 };
    clock_gettime( CLOCK_MONOTONIC, &tp );
    return double(tp.tv_sec) + double(tp.tv_nsec) * .000000001;
#endif
}


static double GetStreamCpuLoad( PaStream* s )
{
    PaAsioStream *stream = (PaAsioStream*)s;

    return PaUtil_GetCpuLoad( &stream->cpuLoadMeasurer );
}


/*
    As separate stream interfaces are used for blocking and callback
    streams, the following functions can be guaranteed to only be called
    for blocking streams.
*/

static PaError ReadStream( PaStream      *s     ,
                           void          *buffer,
                           unsigned long  frames )
{
    PaError result = paNoError; /* Initial return value. */
    PaAsioStream *stream = (PaAsioStream*)s; /* The PA ASIO stream. */

    /* Pointer to the blocking i/o data struct. */
    PaAsioStreamBlockingState *blockingState = stream->blockingState;

    /* Get blocking i/o buffer processor and ring buffer pointers. */
    PaUtilBufferProcessor *pBp = &blockingState->bufferProcessor;
    PaUtilRingBuffer      *pRb = &blockingState->readRingBuffer;

    /* Ring buffer segment(s) used for writing. */
    void *pRingBufferData1st = NULL; /* First segment. (Mandatory) */
    void *pRingBufferData2nd = NULL; /* Second segment. (Optional) */

    /* Number of frames per ring buffer segment. */
    long lRingBufferSize1st = 0; /* First segment. (Mandatory) */
    long lRingBufferSize2nd = 0; /* Second segment. (Optional) */

    /* Get number of frames to be processed per data block. */
    unsigned long lFramesPerBlock = stream->bufferProcessor.framesPerUserBuffer;
    /* Actual number of frames that has been copied into the ring buffer. */
    unsigned long lFramesCopied = 0;
    /* The number of remaining unprocessed dtat frames. */
    unsigned long lFramesRemaining = frames;

    /* Copy the input argument to avoid pointer increment! */
    const void *userBuffer;
    unsigned int i; /* Just a counter. */

    /* About the time, needed to process 8 data blocks. */
    DWORD timeout = (DWORD)( 8 * lFramesPerBlock * 1000 / stream->streamRepresentation.streamInfo.sampleRate );
    DWORD waitResult = 0;


    /* Check if the stream is still available ready to gather new data. */
    if( blockingState->stopFlag || !stream->isActive )
    {
        PA_DEBUG(("Warning! Stream no longer available for reading in ReadStream()\n"));
        result = paStreamIsStopped;
        return result;
    }

    /* If the stream is a input stream. */
    if( stream->inputChannelCount )
    {
        /* Prepare buffer access. */
        if( !pBp->userOutputIsInterleaved )
        {
            userBuffer = blockingState->readStreamBuffer;
            for( i = 0; i<pBp->inputChannelCount; ++i )
            {
                ((void**)userBuffer)[i] = ((void**)buffer)[i];
            }
        } /* Use the unchanged buffer. */
        else { userBuffer = buffer; }

        do /* Internal block processing for too large user data buffers. */
        {
            /* Get the size of the current data block to be processed. */
            lFramesPerBlock =(lFramesPerBlock < lFramesRemaining)
                            ? lFramesPerBlock : lFramesRemaining;
            /* Use predefined block size for as long there are enough
               buffers available, thereafter reduce the processing block
               size to match the number of remaining buffers. So the final
               data block is processed although it may be incomplete. */

            /* If the available amount of data frames is insufficient. */
            if( PaUtil_GetRingBufferReadAvailable(pRb) < (long) lFramesPerBlock )
            {
                /* Make sure, the event isn't already set! */
                /* NspeResetEvent( blockingState->readFramesReadyEvent ); */

                /* Set the number of requested buffers. */
                blockingState->readFramesRequested = lFramesPerBlock;

                /* Signalize that additional buffers are need. */
                blockingState->readFramesRequestedFlag = true;

                /* Wait until requested number of buffers has been freed. */
                waitResult = NspeWaitForEvent( blockingState->readFramesReadyEvent, timeout );

                /* If something seriously went wrong... */
                if( waitResult == WAIT_FAILED )
                {
                    PA_DEBUG(("WaitForEvent() failed in ReadStream()\n"));
                    result = paUnanticipatedHostError;
                    PA_CWASIO_SET_LAST_SYSTEM_ERROR( getLastError() );
                    return result;
                }
                else if( waitResult == WAIT_TIMEOUT )
                {
                    PA_DEBUG(("WaitForEvent() timed out in ReadStream()\n"));

                    /* If block processing has stopped, abort! */
                    if( blockingState->stopFlag ) { return result = paStreamIsStopped; }

                    /* If a timeout is encountered, give up eventually. */
                    return result = paTimedOut;
                }
            }
            /* Now, the ring buffer contains the required amount of data
               frames.
               (Therefor we don't need to check the return argument of
               PaUtil_GetRingBufferReadRegions(). ;-) )
            */

            /* Retrieve pointer(s) to the ring buffer's current write
               position(s). If the first buffer segment is too small to
               store the requested number of bytes, an additional second
               segment is returned. Otherwise, i.e. if the first segment
               is large enough, the second segment's pointer will be NULL.
            */
            PaUtil_GetRingBufferReadRegions(pRb                ,
                                            lFramesPerBlock    ,
                                            &pRingBufferData1st,
                                            &lRingBufferSize1st,
                                            &pRingBufferData2nd,
                                            &lRingBufferSize2nd);

            /* Set number of frames to be copied from the ring buffer. */
            PaUtil_SetInputFrameCount( pBp, lRingBufferSize1st );
            /* Setup ring buffer access. */
            PaUtil_SetInterleavedInputChannels(pBp               ,  /* Buffer processor. */
                                               0                 ,  /* The first channel's index. */
                                               pRingBufferData1st,  /* First ring buffer segment. */
                                               0                 ); /* Use all available channels. */

            /* If a second ring buffer segment is required. */
            if( lRingBufferSize2nd ) {
                /* Set number of frames to be copied from the ring buffer. */
                PaUtil_Set2ndInputFrameCount( pBp, lRingBufferSize2nd );
                /* Setup ring buffer access. */
                PaUtil_Set2ndInterleavedInputChannels(pBp               ,  /* Buffer processor. */
                                                      0                 ,  /* The first channel's index. */
                                                      pRingBufferData2nd,  /* Second ring buffer segment. */
                                                      0                 ); /* Use all available channels. */
            }

            /* Let the buffer processor handle "copy and conversion" and
               update the ring buffer indices manually. */
            lFramesCopied = PaUtil_CopyInput( pBp, &buffer, lFramesPerBlock );
            PaUtil_AdvanceRingBufferReadIndex( pRb, lFramesCopied );

            /* Decrease number of unprocessed frames. */
            lFramesRemaining -= lFramesCopied;

        } /* Continue with the next data chunk. */
        while( lFramesRemaining );


        /* If there has been an input overflow within the callback */
        if( blockingState->inputOverflowFlag )
        {
            blockingState->inputOverflowFlag = false;

            /* Return the corresponding error code. */
            result = paInputOverflowed;
        }

    } /* If this is not an input stream. */
    else {
        result = paCanNotReadFromAnOutputOnlyStream;
    }

    return result;
}

static PaError WriteStream( PaStream      *s     ,
                            const void    *buffer,
                            unsigned long  frames )
{
    PaError result = paNoError; /* Initial return value. */
    PaAsioStream *stream = (PaAsioStream*)s; /* The PA ASIO stream. */

    /* Pointer to the blocking i/o data struct. */
    PaAsioStreamBlockingState *blockingState = stream->blockingState;

    /* Get blocking i/o buffer processor and ring buffer pointers. */
    PaUtilBufferProcessor *pBp = &blockingState->bufferProcessor;
    PaUtilRingBuffer      *pRb = &blockingState->writeRingBuffer;

    /* Ring buffer segment(s) used for writing. */
    void *pRingBufferData1st = NULL; /* First segment. (Mandatory) */
    void *pRingBufferData2nd = NULL; /* Second segment. (Optional) */

    /* Number of frames per ring buffer segment. */
    long lRingBufferSize1st = 0; /* First segment. (Mandatory) */
    long lRingBufferSize2nd = 0; /* Second segment. (Optional) */

    /* Get number of frames to be processed per data block. */
    unsigned long lFramesPerBlock = stream->bufferProcessor.framesPerUserBuffer;
    /* Actual number of frames that has been copied into the ring buffer. */
    unsigned long lFramesCopied = 0;
    /* The number of remaining unprocessed dtat frames. */
    unsigned long lFramesRemaining = frames;

    /* About the time, needed to process 8 data blocks. */
    DWORD timeout = (DWORD)( 8 * lFramesPerBlock * 1000 / stream->streamRepresentation.streamInfo.sampleRate );
    int waitResult = 0;

    /* Copy the input argument to avoid pointer increment! */
    const void *userBuffer;
    unsigned int i; /* Just a counter. */


    /* Check if the stream is still available ready to receive new data. */
    if( blockingState->stopFlag || !stream->isActive )
    {
        PA_DEBUG(("Warning! Stream no longer available for writing in WriteStream()\n"));
        result = paStreamIsStopped;
        return result;
    }

    /* If the stream is a output stream. */
    if( stream->outputChannelCount )
    {
        /* Prepare buffer access. */
        if( !pBp->userOutputIsInterleaved )
        {
            userBuffer = blockingState->writeStreamBuffer;
            for( i = 0; i<pBp->outputChannelCount; ++i )
            {
                ((const void**)userBuffer)[i] = ((const void**)buffer)[i];
            }
        } /* Use the unchanged buffer. */
        else { userBuffer = buffer; }


        do /* Internal block processing for too large user data buffers. */
        {
            /* Get the size of the current data block to be processed. */
            lFramesPerBlock =(lFramesPerBlock < lFramesRemaining)
                            ? lFramesPerBlock : lFramesRemaining;
            /* Use predefined block size for as long there are enough
               frames available, thereafter reduce the processing block
               size to match the number of remaining frames. So the final
               data block is processed although it may be incomplete. */

            /* If the available amount of buffers is insufficient. */
            if( PaUtil_GetRingBufferWriteAvailable(pRb) < (long) lFramesPerBlock )
            {
                /* Make sure, the event isn't already set! */
                /* NspeResetEvent( blockingState->writeBuffersReadyEvent ); */

                /* Set the number of requested buffers. */
                blockingState->writeBuffersRequested = lFramesPerBlock;

                /* Signalize that additional buffers are need. */
                blockingState->writeBuffersRequestedFlag = true;

                /* Wait until requested number of buffers has been freed. */
                waitResult = NspeWaitForEvent( blockingState->writeBuffersReadyEvent, timeout );

                /* If something seriously went wrong... */
                if( waitResult == WAIT_FAILED )
                {
                    PA_DEBUG(("WaitForEvent() failed in WriteStream()\n"));
                    result = paUnanticipatedHostError;
                    PA_CWASIO_SET_LAST_SYSTEM_ERROR( getLastError() );
                    return result;
                }
                else if( waitResult == WAIT_TIMEOUT )
                {
                    PA_DEBUG(("WaitForEvent() timed out in WriteStream()\n"));

                    /* If block processing has stopped, abort! */
                    if( blockingState->stopFlag ) { return result = paStreamIsStopped; }

                    /* If a timeout is encountered, give up eventually. */
                    return result = paTimedOut;
                }
            }
            /* Now, the ring buffer contains the required amount of free
               space to store the provided number of data frames.
               (Therefor we don't need to check the return argument of
               PaUtil_GetRingBufferWriteRegions(). ;-) )
            */

            /* Retrieve pointer(s) to the ring buffer's current write
               position(s). If the first buffer segment is too small to
               store the requested number of bytes, an additional second
               segment is returned. Otherwise, i.e. if the first segment
               is large enough, the second segment's pointer will be NULL.
            */
            PaUtil_GetRingBufferWriteRegions(pRb                ,
                                             lFramesPerBlock    ,
                                             &pRingBufferData1st,
                                             &lRingBufferSize1st,
                                             &pRingBufferData2nd,
                                             &lRingBufferSize2nd);

            /* Set number of frames to be copied to the ring buffer. */
            PaUtil_SetOutputFrameCount( pBp, lRingBufferSize1st );
            /* Setup ring buffer access. */
            PaUtil_SetInterleavedOutputChannels(pBp               ,  /* Buffer processor. */
                                                0                 ,  /* The first channel's index. */
                                                pRingBufferData1st,  /* First ring buffer segment. */
                                                0                 ); /* Use all available channels. */

            /* If a second ring buffer segment is required. */
            if( lRingBufferSize2nd ) {
                /* Set number of frames to be copied to the ring buffer. */
                PaUtil_Set2ndOutputFrameCount( pBp, lRingBufferSize2nd );
                /* Setup ring buffer access. */
                PaUtil_Set2ndInterleavedOutputChannels(pBp               ,  /* Buffer processor. */
                                                       0                 ,  /* The first channel's index. */
                                                       pRingBufferData2nd,  /* Second ring buffer segment. */
                                                       0                 ); /* Use all available channels. */
            }

            /* Let the buffer processor handle "copy and conversion" and
               update the ring buffer indices manually. */
            lFramesCopied = PaUtil_CopyOutput( pBp, &userBuffer, lFramesPerBlock );
            PaUtil_AdvanceRingBufferWriteIndex( pRb, lFramesCopied );

            /* Decrease number of unprocessed frames. */
            lFramesRemaining -= lFramesCopied;

        } /* Continue with the next data chunk. */
        while( lFramesRemaining );


        /* If there has been an output underflow within the callback */
        if( blockingState->outputUnderflowFlag )
        {
            blockingState->outputUnderflowFlag = false;

            /* Return the corresponding error code. */
            result = paOutputUnderflowed;
        }

    } /* If this is not an output stream. */
    else
    {
        result = paCanNotWriteToAnInputOnlyStream;
    }

    return result;
}


static signed long GetStreamReadAvailable( PaStream* s )
{
    PaAsioStream *stream = (PaAsioStream*)s;

    /* Call buffer utility routine to get the number of available frames. */
    return PaUtil_GetRingBufferReadAvailable( &stream->blockingState->readRingBuffer );
}


static signed long GetStreamWriteAvailable( PaStream* s )
{
    PaAsioStream *stream = (PaAsioStream*)s;

    /* Call buffer utility routine to get the number of empty buffers. */
    return PaUtil_GetRingBufferWriteAvailable( &stream->blockingState->writeRingBuffer );
}


/* This routine will be called by the PortAudio engine when audio is needed.
** It may called at interrupt level on some machines so don't do anything
** that could mess up the system like calling malloc() or free().
*/
static int BlockingIoPaCallback(const void                     *inputBuffer    ,
                                      void                     *outputBuffer   ,
                                      unsigned long             framesPerBuffer,
                                const PaStreamCallbackTimeInfo *timeInfo       ,
                                      PaStreamCallbackFlags     statusFlags    ,
                                      void                     *userData       )
{
    PaError result = paNoError; /* Initial return value. */
    PaAsioStream *stream = *(PaAsioStream**)userData; /* The PA ASIO stream. */
    PaAsioStreamBlockingState *blockingState = stream->blockingState; /* Persume blockingState is valid, otherwise the callback wouldn't be running. */

    /* Get a pointer to the stream's blocking i/o buffer processor. */
    PaUtilBufferProcessor *pBp = &blockingState->bufferProcessor;
    PaUtilRingBuffer      *pRb = NULL;

    /* If output data has been requested. */
    if( stream->outputChannelCount )
    {
        /* If the callback input argument signalizes a output underflow,
           make sure the WriteStream() function knows about it, too! */
        if( statusFlags & paOutputUnderflowed ) {
            blockingState->outputUnderflowFlag = true;
        }

        /* Access the corresponding ring buffer. */
        pRb = &blockingState->writeRingBuffer;

        /* If the blocking i/o buffer contains enough output data, */
        if( PaUtil_GetRingBufferReadAvailable(pRb) >= (long) framesPerBuffer )
        {
            /* Extract the requested data from the ring buffer. */
            PaUtil_ReadRingBuffer( pRb, outputBuffer, framesPerBuffer );
        }
        else /* If no output data is available :-( */
        {
            /* Signalize a write-buffer underflow. */
            blockingState->outputUnderflowFlag = true;

            /* Fill the output buffer with silence. */
            (*pBp->outputZeroer)( outputBuffer, 1, pBp->outputChannelCount * framesPerBuffer );

            /* If playback is to be stopped */
            if( blockingState->stopFlag && PaUtil_GetRingBufferReadAvailable(pRb) < (long) framesPerBuffer )
            {
                /* Extract all the remaining data from the ring buffer,
                   whether it is a complete data block or not. */
                PaUtil_ReadRingBuffer( pRb, outputBuffer, PaUtil_GetRingBufferReadAvailable(pRb) );
            }
        }

        /* Set blocking i/o event? */
        if( blockingState->writeBuffersRequestedFlag && PaUtil_GetRingBufferWriteAvailable(pRb) >= (long) blockingState->writeBuffersRequested )
        {
            /* Reset buffer request. */
            blockingState->writeBuffersRequestedFlag = false;
            blockingState->writeBuffersRequested     = 0;
            /* Signalize that requested buffers are ready. */
            NspeSetEvent( blockingState->writeBuffersReadyEvent );
            /* What do we do if SetEvent() returns zero, i.e. the event
               could not be set? How to return errors from within the
               callback? - S.Fischer */
        }
    }

    /* If input data has been supplied. */
    if( stream->inputChannelCount )
    {
        /* If the callback input argument signalizes a input overflow,
           make sure the ReadStream() function knows about it, too! */
        if( statusFlags & paInputOverflowed ) {
            blockingState->inputOverflowFlag = true;
        }

        /* Access the corresponding ring buffer. */
        pRb = &blockingState->readRingBuffer;

        /* If the blocking i/o buffer contains not enough input buffers */
        if( PaUtil_GetRingBufferWriteAvailable(pRb) < (long) framesPerBuffer )
        {
            /* Signalize a read-buffer overflow. */
            blockingState->inputOverflowFlag = true;

            /* Remove some old data frames from the buffer. */
            PaUtil_AdvanceRingBufferReadIndex( pRb, framesPerBuffer );
        }

        /* Insert the current input data into the ring buffer. */
        PaUtil_WriteRingBuffer( pRb, inputBuffer, framesPerBuffer );

        /* Set blocking i/o event? */
        if( blockingState->readFramesRequestedFlag && PaUtil_GetRingBufferReadAvailable(pRb) >= (long) blockingState->readFramesRequested )
        {
            /* Reset buffer request. */
            blockingState->readFramesRequestedFlag = false;
            blockingState->readFramesRequested     = 0;
            /* Signalize that requested buffers are ready. */
            NspeSetEvent( blockingState->readFramesReadyEvent );
            /* What do we do if SetEvent() returns zero, i.e. the event
               could not be set? How to return errors from within the
               callback? - S.Fischer */
            /** @todo report an error with PA_DEBUG */
        }
    }

    return paContinue;
}


PaError PaCwAsio_ShowControlPanel( PaDeviceIndex device, void* systemSpecific )
{
    PaError result = paNoError;
    PaUtilHostApiRepresentation *hostApi;
    PaDeviceIndex hostApiDevice;
    struct cwASIODriverInfo asioDriverInfo;
    cwASIOError asioError;
    int asioIsLoaded = 0;
    int asioIsInitialized = 0;
    PaCwAsioHostApiRepresentation *cwAsioHostApi;
    PaCwAsioDeviceInfo *cwAsioDeviceInfo;
    char clsid[64]; *clsid = '\0';

    /*
        COM will be handled correctly by cwASIO. No need for us to take care about this.
    */
    result = PaUtil_GetHostApiRepresentation( &hostApi, paASIO );
    if( result != paNoError )
        goto error;

    result = PaUtil_DeviceIndexToHostApiDeviceIndex( &hostApiDevice, device, hostApi );
    if( result != paNoError )
        goto error;

    /*
        In theory we could proceed if the currently open device was the same
        one for which the control panel was requested, however  because the
        window pointer is not available until this function is called we
        currently need to call cwASIOInit() again here, which of course can't be
        done safely while a stream is open.
    */

    cwAsioHostApi = (PaCwAsioHostApiRepresentation*)hostApi;
    if( cwAsioHostApi->openAsioDeviceIndex != paNoDevice )
    {
        result = paDeviceUnavailable;
        goto error;
    }

    cwAsioDeviceInfo = (PaCwAsioDeviceInfo*)hostApi->deviceInfos[hostApiDevice];
    result = getDriverClsid( cwAsioHostApi, cwAsioDeviceInfo->commonDeviceInfo.name, clsid, sizeof(clsid) );
    if (result != paNoError || *clsid == '\0')
    {
        result = paUnanticipatedHostError;
        goto error;
    }

    if( cwASIOLoad( clsid ) != ASE_OK )
    {
        result = paUnanticipatedHostError;
        goto error;
    }
    else
    {
        asioIsLoaded = 1;
    }

    /* CRUCIAL!!! */
    memset( &asioDriverInfo, 0, sizeof(struct cwASIODriverInfo) );
    asioDriverInfo.asioVersion = 2;
    asioDriverInfo.sysRef = systemSpecific;
    asioError = cwASIOInit( &asioDriverInfo );
    if( asioError != ASE_OK )
    {
        result = paUnanticipatedHostError;
        PA_CWASIO_SET_LAST_ASIO_ERROR( asioError );
        goto error;
    }
    else
    {
        asioIsInitialized = 1;
    }

PA_DEBUG(("PaAsio_ShowControlPanel: cwASIOInit(): %s\n", PaAsio_GetAsioErrorText(asioError) ));
PA_DEBUG(("asioVersion: cwASIOInit(): %ld\n",   asioDriverInfo.asioVersion ));
PA_DEBUG(("driverVersion: cwASIOInit(): %ld\n", asioDriverInfo.driverVersion ));
PA_DEBUG(("Name: cwASIOInit(): %s\n",           asioDriverInfo.name ));
PA_DEBUG(("ErrorMessage: cwASIOInit(): %s\n",   asioDriverInfo.errorMessage ));

    asioError = cwASIOControlPanel();
    if( asioError != ASE_OK )
    {
        PA_DEBUG(("PaAsio_ShowControlPanel: ASIOControlPanel(): %s\n", PaAsio_GetAsioErrorText(asioError) ));
        result = paUnanticipatedHostError;
        PA_CWASIO_SET_LAST_ASIO_ERROR( asioError );
        goto error;
    }

PA_DEBUG(("PaAsio_ShowControlPanel: ASIOControlPanel(): %s\n", PaAsio_GetAsioErrorText(asioError) ));

    asioError = cwASIOExit();
    if( asioError != ASE_OK )
    {
        result = paUnanticipatedHostError;
        PA_CWASIO_SET_LAST_ASIO_ERROR( asioError );
        asioIsInitialized = 0;
        goto error;
    }

PA_DEBUG(("PaAsio_ShowControlPanel: cwASIOExit(): %s\n", PaAsio_GetAsioErrorText(asioError) ));

    cwASIOUnload();

PA_DEBUG(("PaAsio_ShowControlPanel: cwASIOUnload()\n" ));

    return result;

error:
    if( asioIsInitialized )
    {
        cwASIOExit();
    }
    if (asioIsLoaded)
    {
        cwASIOUnload();
    }

    return result;
}


PaError PaCwAsio_GetInputChannelName( PaDeviceIndex device, int channelIndex,
        const char** channelName )
{
    PaError result = paNoError;
    PaUtilHostApiRepresentation *hostApi;
    PaDeviceIndex hostApiDevice;
    PaCwAsioDeviceInfo *cwAsioDeviceInfo;


    result = PaUtil_GetHostApiRepresentation( &hostApi, paASIO );
    if( result != paNoError )
        goto error;

    result = PaUtil_DeviceIndexToHostApiDeviceIndex( &hostApiDevice, device, hostApi );
    if( result != paNoError )
        goto error;

    cwAsioDeviceInfo = (PaCwAsioDeviceInfo*)hostApi->deviceInfos[hostApiDevice];

    if( channelIndex < 0 || channelIndex >= cwAsioDeviceInfo->commonDeviceInfo.maxInputChannels ){
        result = paInvalidChannelCount;
        goto error;
    }

    *channelName = cwAsioDeviceInfo->asioChannelInfos[channelIndex].name;

    return paNoError;

error:
    return result;
}


PaError PaCwAsio_GetOutputChannelName( PaDeviceIndex device, int channelIndex,
        const char** channelName )
{
    PaError result = paNoError;
    PaUtilHostApiRepresentation *hostApi;
    PaDeviceIndex hostApiDevice;
    PaCwAsioDeviceInfo * cwAsioDeviceInfo;


    result = PaUtil_GetHostApiRepresentation( &hostApi, paASIO );
    if( result != paNoError )
        goto error;

    result = PaUtil_DeviceIndexToHostApiDeviceIndex( &hostApiDevice, device, hostApi );
    if( result != paNoError )
        goto error;

    cwAsioDeviceInfo = (PaCwAsioDeviceInfo*)hostApi->deviceInfos[hostApiDevice];

    if( channelIndex < 0 || channelIndex >= cwAsioDeviceInfo->commonDeviceInfo.maxOutputChannels ){
        result = paInvalidChannelCount;
        goto error;
    }

    *channelName = cwAsioDeviceInfo->asioChannelInfos[
        cwAsioDeviceInfo->commonDeviceInfo.maxInputChannels + channelIndex].name;

    return paNoError;

error:
    return result;
}


/* NOTE: the following functions are ASIO-stream specific, and are called directly
    by client code. We need to check for many more error conditions here because
    we don't have the benefit of pa_front.c's parameter checking.
*/

static PaError GetAsioStreamPointer( PaAsioStream **stream, PaStream *s )
{
    PaError result;
    PaUtilHostApiRepresentation *hostApi;
    PaCwAsioHostApiRepresentation *cwAsioHostApi;

    result = PaUtil_ValidateStreamPointer( s );
    if( result != paNoError )
        return result;

    result = PaUtil_GetHostApiRepresentation( &hostApi, paASIO );
    if( result != paNoError )
        return result;

    cwAsioHostApi = (PaCwAsioHostApiRepresentation*)hostApi;

    if( PA_STREAM_REP( s )->streamInterface == &cwAsioHostApi->callbackStreamInterface
            || PA_STREAM_REP( s )->streamInterface == &cwAsioHostApi->blockingStreamInterface )
    {
        /* s is an ASIO  stream */
        *stream = (PaAsioStream *)s;
        return paNoError;
    }
    else
    {
        return paIncompatibleStreamHostApi;
    }
}


PaError PaCwAsio_SetStreamSampleRate( PaStream* s, double sampleRate )
{
    PaAsioStream *stream;
    PaError result = GetAsioStreamPointer( &stream, s );
    if( result != paNoError )
        return result;

    if( stream != theAsioStream )
        return paBadStreamPtr;

    return ValidateAndSetSampleRate( sampleRate );
}
