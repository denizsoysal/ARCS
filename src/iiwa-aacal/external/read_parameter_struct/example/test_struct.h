#ifndef TEST_STRUCT_H
#define TEST_STRUCT_H

/* Generic sructure */
typedef struct param_x_s{
    int         a1;
    int         a2;
    int         a3;

    float       b1;
    float       b2;
    float       b3;

    char*       c1;
    char*       c2;
    char*       c3;

    struct array_s{
        int     d1;
        float   d2;
        char*   d3;

        struct arrayception_s{
            int     e1;
            float   e2;
            char*   e3;
        }*arrayception;
    }*array;

    /* Mandatory string pointing to input file */
    const char* param_file_path;
}param_x_t;

/* User written function pointing where to write which data */
int read_params_x(param_x_t* param_x);

#endif /* TEST_STRUCT_H */