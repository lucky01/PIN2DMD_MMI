/*
 * log.c
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
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h> // time_t, tm, time, localtime, strftime

typedef struct _liveLogEntry {
	char* msg;
	unsigned long timeInMillis;
	int inc;
} LiveLogEntry;

#define LIVELOG_SIZE 100

int logLevel = 4;
const char* logLevelLabel[6] = { "", "ERROR", "WARN", "INFO", "DEBUG", "TRACE" };

FILE* logdev = NULL;

volatile int liveLogLevel = 0;
LiveLogEntry liveLog[LIVELOG_SIZE];
char* liveLogFilter = NULL;
int liveLogHead = 0;
int liveLogInc = 1;

/**
 * set loglevel for live logging. could be different from normal logging
 */
void setLiveLogLevel( int level ) {
	if( level >= 0 && level <= 5 )
		liveLogLevel = level;
}

/**
 * sets a filter expression for live logging.
 * filter must be included in the log line
 */
void setLiveLogFilter( const char* filter ) {
	if( liveLogFilter )
		free( liveLogFilter );
	liveLogFilter = filter ? strdup( filter ) : NULL;
}

/**
 * set logLevel for logging
 */
void setLogLevel( int lev ) { logLevel = lev; }

/**
 * insert into ring buffer for live log
 */
void insertIntoLiveLog( const char* msg, unsigned long now ) {
	if( ( liveLogFilter == NULL || strstr( msg, liveLogFilter ) ) &&
	    strstr( msg, "/livelog" ) == NULL ) { // filter out livelog requests
		if( liveLog[liveLogHead].msg )
			free( liveLog[liveLogHead].msg );
		liveLog[liveLogHead].msg = strdup( msg );
		liveLog[liveLogHead].timeInMillis = now;
		liveLog[liveLogHead].inc = liveLogInc++;
		liveLogHead++;
		if( liveLogHead >= LIVELOG_SIZE )
			liveLogHead = 0;
	}
}
static unsigned long startTime;

unsigned long timeInMillis() {
	struct timeval tv;
	gettimeofday( &tv, NULL );
	return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

/**
 * sets logdevice whene to write log output.
 */
void setLogDevice( FILE* f ) {
	logdev = f;
	startTime = timeInMillis();
}

void flushLog() {
	if( logdev )
		fflush( logdev );
}

// Returns the local date/time formatted as 2014-03-19 11:11:52
char* getFormattedTime( void ) {

	time_t rawtime;
	struct tm* timeinfo;

	time( &rawtime );
	timeinfo = localtime( &rawtime );

	// Must be static, otherwise won't work
	static char _retval[25] = { '\0' };
#ifndef LOG_MILLIS
	strftime( _retval, sizeof( _retval ), "%Y-%m-%d %H:%M:%S", timeinfo );
#endif
	char* p = _retval + strlen( _retval );
	struct timeval tv;
	gettimeofday( &tv, NULL );
#ifdef LOG_MILLIS
	sprintf( _retval, "%06d", (long)( ( tv.tv_sec * 1000 + ( tv.tv_usec / 1000 ) - startTime ) % 100000 ) );
#else
	sprintf( p, ".%03d", (int)( ( tv.tv_usec / 1000 ) % 1000 ) );
#endif
	return _retval;
}

int getThreadId() { return (int)pthread_self(); }

char* getThreadName( char* buf, size_t buflen ) {
	pthread_getname_np( pthread_self(), buf, buflen );
	return buf;
}

void setThreadName( const char* name ) {
#if defined( _GNU_SOURCE ) | defined( WINDOWS )
	pthread_setname_np( pthread_self(), name );
#else
	pthread_setname_np( name );
#endif
}
