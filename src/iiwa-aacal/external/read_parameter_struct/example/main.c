#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "test_struct.h"

int main(int argc, char *argv[])
{
    printf("Starting\n");

    /* Assign memory for pointers */
    param_x_t* param_x = malloc(sizeof(param_x_t));
    param_x->c1 = malloc(sizeof(char)*256);
    param_x->c2 = malloc(sizeof(char)*256);
    param_x->c3 = malloc(sizeof(char)*256);
    param_x->array = malloc(sizeof(struct array_s));
    param_x->array->d3 = malloc(sizeof(char)*256);
    param_x->array->arrayception = malloc(sizeof(struct arrayception_s));
    param_x->array->arrayception->e3 = malloc(sizeof(char)*256);

    /* Write path to input file to structure */    
    param_x->param_file_path = "input_test.json";

    /* Do actual reading */     
    if(read_params_x(param_x) == 1)
    {
        /* Print to show results */
        printf("a1 %i\n",param_x->a1);
        printf("a2 %i\n",param_x->a2);
        printf("a3 %i\n",param_x->a3);

        printf("b1 %f\n",param_x->b1);
        printf("b2 %f\n",param_x->b2);
        printf("b3 %f\n",param_x->b3);

        printf("c1 %s\n",param_x->c1);
        printf("c2 %s\n",param_x->c2);
        printf("c3 %s\n",param_x->c3);

        printf("d1 %i\n",param_x->array->d1);
        printf("d2 %f\n",param_x->array->d2);
        printf("d3 %s\n",param_x->array->d3);

        printf("e1 %i\n",param_x->array->arrayception->e1);
        printf("e2 %f\n",param_x->array->arrayception->e2);
        printf("e3 %s\n",param_x->array->arrayception->e3);

        printf("Finished Succesfully!\n");
        return EXIT_SUCCESS;
    }
    else
    {
        printf("Finished Unsuccesfully!\n");
        return EXIT_SUCCESS;
    }
}
