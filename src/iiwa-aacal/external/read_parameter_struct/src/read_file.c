#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <read_file/read_file.h>
#include <cJSON.h>

#ifndef NAN
#define NAN 0.0/0.0
#endif


int read_from_json_file(const char *filename, param_array_t param_array[], int length_array)
{
    /* Open file */
    FILE *infile = fopen (filename, "r");
    if (infile == NULL)
    {
        printf("ERROR: Could not open '%s' input file\n", filename);
        return CONFIGURATION_FROM_FILE_FAILED;
    }

    char buffer[2048];
    fread(buffer, 1, sizeof buffer - 1, infile);
    fclose (infile);
    cJSON *root = cJSON_Parse(buffer);
    if (root == NULL)
    {
        printf("ERROR: Could not interpret '%s'!\n", filename);
        cJSON_Delete(root);
        return CONFIGURATION_FROM_FILE_FAILED;
    }
    
    cJSON *element;
    int i = 0;
    for(i; i < length_array;i++) 
    {
        char foo[strlen(param_array[i].param_name)+1];
        strcpy(foo,param_array[i].param_name);
        char* ch_pos = strchr(foo,'/');
        char subelement[sizeof(foo)];
        memset(subelement,0,sizeof(foo));
        element = root;

        while (ch_pos!=NULL)
        {
            /* GO DEEPA */
            memcpy(subelement, foo, ch_pos-foo);
            element = cJSON_GetObjectItemCaseSensitive(element, subelement);

            int len =  ch_pos-foo+1;
            memmove(foo, foo +len, sizeof(foo)-len);
            memset(subelement,0,sizeof(foo));
            ch_pos = strchr(foo,'/');
        }
        memcpy(subelement, foo, sizeof(foo));
        element = cJSON_GetObjectItemCaseSensitive(element, subelement);
        
        if(param_array[i].param_type == PARAM_TYPE_INT)
        {
            if (element->valueint == NAN)
                {goto ErrorRead;}
            else
                {*((int*) param_array[i].param_pointer) = element->valueint;}
        }
        else if (param_array[i].param_type == PARAM_TYPE_FLOAT)
        {
            if (element->valuedouble == NAN)
                {goto ErrorRead;}
            *((float*) param_array[i].param_pointer) = element->valuedouble;
        }
        else if (param_array[i].param_type == PARAM_TYPE_DOUBLE)
        {
            if (element->valuedouble == NAN)
                {goto ErrorRead;}
            *((double*) param_array[i].param_pointer) = element->valuedouble;
        }
        else if (param_array[i].param_type == PARAM_TYPE_CHAR)
        {
            if (element->valuestring == NULL)
                {goto ErrorRead;}
            strcpy((char*) param_array[i].param_pointer, (const char*) element->valuestring);
        }
        else
        {
            printf("ERROR: Parameter %s does not have a valid parameter type specified!\n", param_array[i].param_name);
            goto ErrorRead;
        }
    }

    cJSON_Delete(root);
    return CONFIGURATION_FROM_FILE_SUCCEEDED;

ErrorRead:
    printf("Could not read '%s' from input file: '%s'\n", param_array[i].param_name, filename);
    cJSON_Delete(root);
    return CONFIGURATION_FROM_FILE_FAILED;
}


void read_from_input_file(const char *filename, param_array_t param_array[], int length_array, int *status)
{
    const char* extension = strrchr(filename, '.');

    if(strcmp(extension, ".json") == 0)
    {
        *status = read_from_json_file(filename, param_array, length_array);
    }
    else if(strcmp(extension, ".xml") == 0)
    {
        /* TODO */
        // *status = read_from_XML_file(filename, param_array, length_array);
    }
    else
    {
        printf("ERROR: Input file '%s' has an invalid extension\n", filename);
        *status = CONFIGURATION_FROM_FILE_FAILED;
    }
}
