// JsonForWaterMusic.cpp : Defines the entry point for the console application.
//
#include "JsonParse.h"
#include <string.h>

//int main()
//{
//    Name name;
//    strcpy(name.name, "ABCD");
//    char resultName[300];
//    NamePareJson(name, resultName);
//    printf("Name %s\n", resultName);

//    char resultSpectrum[300];
//    Spectrum spectrum;

//    for (int i = 0; i < 8; i++) {
//        spectrum.listSpectrum[i] = 100 + i;
//    }

//    SpectrumParseJson(spectrum, resultSpectrum);
//    printf("Name %s\n", resultSpectrum);

//    char result[300];
//    Message message;
//    strcpy(message.type, "Spectrum");
//    strcpy(message.message, resultSpectrum);
//    MessageParseJson(message,result);
//    printf("Name %s\n", result);

//    return 0;
//}

void NameParseJson(Name name, char* result) {
    strcpy(result, "");
    strcat(result, "{\\\"name\\\":\\\"");
    strcat(result, name.name);
    strcat(result, "\\\"}");
};

void SpectrumParseJson(Spectrum spectrum, char* result) {
    strcpy(result, "");
    strcat(result, "{\\\"spectrum\\\":[");
    for (int i = 0; i < 8; i++) {
        char temp[50];
        sprintf(temp, "%d", spectrum.listSpectrum[i]);
        strcat(result, temp);
        if (i != 7)
            strcat(result, ",");
        else {
            strcat(result, "]");
        }
    }
    strcat(result, "}");
};

void MessageParseJson(Message message, char* result) {
    strcpy(result, "");
    strcat(result, "{\"type\":\"");
    strcat(result, message.type);
    strcat(result, "\",\"message\":\"");
    strcat(result, message.message);
    strcat(result, "\"}");
};