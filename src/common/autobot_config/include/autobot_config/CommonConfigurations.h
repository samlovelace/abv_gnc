#pragma once

struct HeartbeatConfig
{
    double mRate;         // Hz - how often each node publishes its heartbeat
    double mStaleAfter;   // seconds - GUI-side threshold before a node is considered disconnected
    int mPingIntervalMs;  // GUI-side: how often to ping the robot host for the Comms indicator
};

struct TableViewConfig
{
    double mWidth;       // meters - table X extent, GUI-side top-down view
    double mHeight;      // meters - table Y extent
    double mRobotWidth;  // meters - robot footprint used to draw the glyph
    double mRobotLength; // meters
};
