#include <string.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct Name {
    char name[255];
}Name;

typedef struct Spectrum {
    int listSpectrum[8];
}Spectrum;

typedef struct Message {
    char type[20];
    char message[300];
}Message;

void NameParseJson(Name name,char* result);
void SpectrumParseJson(Spectrum spectrum, char* result);
void MessageParseJson(Message message, char* result);
