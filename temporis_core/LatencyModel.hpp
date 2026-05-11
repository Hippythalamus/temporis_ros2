#pragma once

class LatencyModel {
public:
    virtual double sample(int sender, int receiver, double t, int network_load, int queue_size) = 0;
    virtual ~LatencyModel() = default;
};