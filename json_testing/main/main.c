#include <stdio.h>
#include <string.h>
#include "cJSON.h"

float test_data = 22.65;
char test[] = "data";

void app_main(void)
{
    // Parse the JSON string
    const char *json_string = "{\"name\":\"John Smith\",\"age\":30,\"city\":\"New York\"}";
    cJSON *root = cJSON_Parse(json_string);

    // Modify the JSON data
    cJSON_ReplaceItemInObject(root, "age", cJSON_CreateNumber(test_data));
    cJSON_ReplaceItemInObject(root, "city", cJSON_CreateString(test));

    
    // Convert the JSON object to a string
    char *new_json_string = cJSON_Print(root);
    size_t size = strlen(json_string);

    // Print the new JSON string to the console
    printf("%s\n", new_json_string);
    printf("%d\n", size);

    // Free the cJSON objects and the JSON strings
    cJSON_Delete(root);
    free(new_json_string);
    printf("End of program\n");
}
