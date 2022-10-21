#ifndef READ_FILE_H
#define READ_FILE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Supported variable types */
#define PARAM_TYPE_INT      1
#define PARAM_TYPE_FLOAT    2
#define PARAM_TYPE_CHAR     3
#define PARAM_TYPE_DOUBLE   4

/* Configuration status */
#define CONFIGURATION_FROM_FILE_FAILED 0
#define CONFIGURATION_FROM_FILE_SUCCEEDED 1

/* Struct to retrieve a single parameter from input file */
typedef struct param_array_s{
    const char  *param_name;    // Path to the to be read parameter inside the input file
    void        *param_pointer; // Pointer where read data should be written to 
    int         param_type;     // Specify the data type of parameter 
}param_array_t;

/* 
    Detects file extension and reads from input file
    @param filename = path to the input file
    @param param_array[] = array of param_array_t struct, data to be read
    @param length_array = amount of elements in param_array[]
    @param status indicates success/fail (see configuration status)
*/
void read_from_input_file(const char *filename, param_array_t param_array[], int length_array, int *status);

/* 
    Function to read from .json file. Called by read_from_input_file() function.
    @param filename = path to the input file
    @param param_array[] = array of param_array_t struct, data to be read
    @param length_array = amount of elements in param_array[]
*/
int read_from_json_file(const char *filename, param_array_t param_array[], int length_array);


#ifdef __cplusplus
}
#endif

#endif /* READ_FILE_H */