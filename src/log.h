/*
 * log.h
 *
 *  Created on: 10.03.2017
 *      Author: Stefan Rinke
 *
 *      (C) 2017-2022 by Stefan Rinke / Rinke Solutions
 *  This work is licensed under a Creative
 *	Commons Attribution-NonCommercial-
 *	ShareAlike 4.0 International License.
 *
 *	http://creativecommons.org/licenses/by-nc-sa/4.0/
 */

#pragma once

#include <stdio.h>
#include <string.h>

//#define LOGDEV stderr
#define LOGENABLED 1

extern const char* logLevelLabel[6];

typedef struct _liveLogEntry {
	char* msg;
	unsigned long timeInMillis;
	int inc;
} LiveLogEntry;

typedef enum { LEVEL_NONE, LEVEL_ERROR, LEVEL_WARN, LEVEL_INFO, LEVEL_DEBUG, LEVEL_TRACE } LOGLEVEL;

#define LIVELOG_SIZE 100
extern LiveLogEntry liveLog[LIVELOG_SIZE];
extern int liveLogHead;
extern volatile int liveLogLevel;
extern char* liveLogFilter;

// extern still necessary because LOGXX macro is directly referencing logLevel var
extern int logLevel;
extern FILE* logdev;

// Returns the local date/time formatted as 2014-03-19 11:11:52
char* getFormattedTime( void );
int getThreadId();
char* getThreadName( char* buf, size_t buflen );
void setThreadName( const char* name );
void setLogDevice( FILE* f );
void flushLog();
void insertIntoLiveLog( const char* msg, unsigned long now );
void setLiveLogLevel( int level );
void setLiveLogFilter( const char* f );
unsigned long timeInMillis();
void setLogLevel( int lev );

// Remove path from filename
#define __SHORT_FILE__ ( strrchr( __FILE__, '/' ) ? strrchr( __FILE__, '/' ) + 1 : __FILE__ )

#define LOGDEV logdev

#define LOG_DEVICE_STDOUT stdout
#define LOG_DEVICE_STDERR stderr

#if LOGENABLED
#ifndef LOGDEV
// Main log macro
#define __LOG__( format, loglevel, ... )                                                                               \
	if( loglevel <= logLevel ) {                                                                                         \
		char t[40];                                                                                                        \
		printf( "%s %-5s %8.8s [%s] [%s:%d] " format "\n", getFormattedTime(), logLevelLabel[loglevel],                    \
		        getThreadName( t, 40 ), __func__, __SHORT_FILE__, __LINE__, ##__VA_ARGS__ );                               \
	}
#else
#define __LOG__( format, loglevel, ... )                                                                               \
	if( loglevel <= logLevel ) {                                                                                         \
		char t[40];                                                                                                        \
		fprintf( LOGDEV, "%s %-5s %8.8s [%s] [%s:%d] " format "\n", getFormattedTime(), logLevelLabel[loglevel],           \
		         getThreadName( t, 40 ), __func__, __SHORT_FILE__, __LINE__, ##__VA_ARGS__ );                              \
		if( loglevel <= liveLogLevel ) {                                                                                   \
			char logline[1024];                                                                                              \
			char t[40];                                                                                                      \
			snprintf( logline, 1024, "%s %-5s %8.8s [%s] [%s:%d] " format "\n", getFormattedTime(), logLevelLabel[loglevel], \
			          getThreadName( t, 40 ), __func__, __SHORT_FILE__, __LINE__, ##__VA_ARGS__ );                           \
			insertIntoLiveLog( logline, timeInMillis() );                                                                    \
		}                                                                                                                  \
	}
#endif
#else
#define __LOG__( format, loglevel, ... )
#endif
// Specific log macros with
#define LOGTRACE( format, ... ) __LOG__( format, 5, ##__VA_ARGS__ )
#define LOGDEBUG( format, ... ) __LOG__( format, 4, ##__VA_ARGS__ )
#define LOGWARN( format, ... ) __LOG__( format, 2, ##__VA_ARGS__ )
#define LOGERROR( format, ... ) __LOG__( format, 1, ##__VA_ARGS__ )
#define LOGINFO( format, ... ) __LOG__( format, 3, ##__VA_ARGS__ )
