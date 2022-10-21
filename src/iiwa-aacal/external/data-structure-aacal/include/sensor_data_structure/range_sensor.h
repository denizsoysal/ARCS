#ifndef RANGE_SENSOR_DATA_STRUCTURE_H
#define RANGE_SENSOR_DATA_STRUCTURE_H

typedef struct range_sensor_s{
    double min_angle;
    double max_angle;
    double min_distance;
    double max_distance;
    double angular_resolution;
    int nb_measurements;
}range_sensor_t;

#endif
