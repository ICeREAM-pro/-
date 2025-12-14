/**
 * @file visualization.cpp
 * @brief EasyX Visualization Program for Differential Drive Robot
 *
 * Compilation Notes:
 * 1. Install EasyX Graphics Library
 * 2. Set Project Character Set to Unicode
 * 3. File Encoding: UTF-8 with BOM
 */

#include <graphics.h>
#include "kinematics.h"
#include "config.h"
#include <stdio.h>
#include <math.h>
#include <vector>
#include <Windows.h>

using namespace std;

// Window Configuration
#define WINDOW_WIDTH 1200
#define WINDOW_HEIGHT 700

// Left and Right Areas (Swapped: Left=Charts, Right=Robot)
#define LEFT_AREA_WIDTH 500
#define RIGHT_AREA_X 500

// Robot Display Scale
#define ROBOT_SCALE 5.0f
#define ROBOT_CENTER_X 850
#define ROBOT_CENTER_Y 350

// Chart Configuration
#define CHART_PADDING 50
#define CHART_WIDTH 400
#define CHART_HEIGHT 250
#define MAX_DATA_POINTS 200

// Color Definitions - Dark Theme
#define COLOR_BG RGB(15, 15, 15)
#define COLOR_ROBOT RGB(120, 140, 160)
#define COLOR_RED_CENTER RGB(255, 80, 80)
#define COLOR_BLUE_CENTER RGB(80, 180, 255)
#define COLOR_WHEEL RGB(180, 180, 200)
#define COLOR_V_L RGB(50, 255, 100)
#define COLOR_V_R RGB(100, 150, 255)
#define COLOR_OFFSET RGB(255, 100, 150)
#define COLOR_GRID RGB(50, 50, 50)
#define COLOR_TEXT RGB(220, 220, 220)

// Data Recording Structure
struct VisData {
    float time;
    float V_L;
    float V_R;
    float offset_d;
    float angle;
};

// Test Case Structure
struct TestCase {
    float initial_offset;
    TCHAR name[50];
    TCHAR desc[100];
};

// Global Variables
TestCase g_tests[4];
int g_currentTest = 0;
vector<VisData> g_dataLog;
bool g_isRunning = false;
bool g_isPaused = false;
float g_simTime = 0.0f;

// Function Declarations
void initTestCases();
void drawBackground();
void drawRobot(float angle, float offset_d);
void drawSpeedChart();
void drawOffsetChart();
void drawInfoPanel();
void drawInstructions();
void runSimulation();
void resetSimulation();
void addDataPoint(float t, float vl, float vr, float od, float a);

// Main Function
int main() {
    // Initialize Test Cases
    initTestCases();

    // Create Window
    initgraph(WINDOW_WIDTH, WINDOW_HEIGHT);
    SetWindowText(GetHWnd(), TEXT("Differential Drive Robot - Rotation Center Control"));

    setbkcolor(COLOR_BG);
    cleardevice();

    BeginBatchDraw();

    bool quit = false;
    while (!quit) {
        // Draw Interface
        cleardevice();
        drawBackground();

        float current_offset = g_tests[g_currentTest].initial_offset;
        if (!g_dataLog.empty()) {
            current_offset = g_dataLog.back().offset_d;
        }

        drawRobot(g_simTime * 0.333f, current_offset);
        drawSpeedChart();
        drawOffsetChart();
        drawInfoPanel();
        drawInstructions();

        FlushBatchDraw();

        // Run Simulation
        if (g_isRunning && !g_isPaused) {
            runSimulation();
            Sleep(50);
        }

        // Keyboard Input - Using GetAsyncKeyState for better reliability
        static bool spacePressed = false;
        static bool rPressed = false;
        static bool nPressed = false;
        static bool pPressed = false;
        static bool escPressed = false;
        static bool qPressed = false;

        // SPACE key
        if (GetAsyncKeyState(VK_SPACE) & 0x8000) {
            if (!spacePressed) {
                if (!g_isRunning) {
                    g_isRunning = true;
                    g_isPaused = false;
                } else {
                    g_isPaused = !g_isPaused;
                }
                spacePressed = true;
            }
        } else {
            spacePressed = false;
        }

        // R key
        if (GetAsyncKeyState('R') & 0x8000) {
            if (!rPressed) {
                resetSimulation();
                rPressed = true;
            }
        } else {
            rPressed = false;
        }

        // N key
        if (GetAsyncKeyState('N') & 0x8000) {
            if (!nPressed) {
                g_currentTest = (g_currentTest + 1) % 4;
                resetSimulation();
                nPressed = true;
            }
        } else {
            nPressed = false;
        }

        // P key
        if (GetAsyncKeyState('P') & 0x8000) {
            if (!pPressed) {
                g_currentTest = (g_currentTest - 1 + 4) % 4;
                resetSimulation();
                pPressed = true;
            }
        } else {
            pPressed = false;
        }

        // ESC key
        if (GetAsyncKeyState(VK_ESCAPE) & 0x8000) {
            if (!escPressed) {
                quit = true;
                escPressed = true;
            }
        } else {
            escPressed = false;
        }

        // Q key
        if (GetAsyncKeyState('Q') & 0x8000) {
            if (!qPressed) {
                quit = true;
                qPressed = true;
            }
        } else {
            qPressed = false;
        }
    }

    EndBatchDraw();
    closegraph();
    return 0;
}

// Initialize Test Cases
void initTestCases() {
    g_tests[0].initial_offset = 0.0f;
    _tcscpy_s(g_tests[0].name, TEXT("Test Case 1"));
    _tcscpy_s(g_tests[0].desc, TEXT("No Initial Offset (delta_d=0)"));

    g_tests[1].initial_offset = 3.0f;
    _tcscpy_s(g_tests[1].name, TEXT("Test Case 2"));
    _tcscpy_s(g_tests[1].desc, TEXT("Initial Offset Right +3.0cm"));

    g_tests[2].initial_offset = -2.0f;
    _tcscpy_s(g_tests[2].name, TEXT("Test Case 3"));
    _tcscpy_s(g_tests[2].desc, TEXT("Initial Offset Left -2.0cm"));

    g_tests[3].initial_offset = 5.0f;
    _tcscpy_s(g_tests[3].name, TEXT("Test Case 4"));
    _tcscpy_s(g_tests[3].desc, TEXT("Initial Offset Right +5.0cm"));
}

// Draw Background
void drawBackground() {
    setlinecolor(RGB(80, 80, 80));
    setlinestyle(PS_SOLID, 2);
    line(LEFT_AREA_WIDTH, 0, LEFT_AREA_WIDTH, WINDOW_HEIGHT);

    settextcolor(COLOR_TEXT);
    settextstyle(24, 0, TEXT("Microsoft YaHei"));
    outtextxy(20, 20, TEXT("Real-time Data"));
    outtextxy(RIGHT_AREA_X + 20, 20, TEXT("Robot Animation"));
}

// Draw Robot
void drawRobot(float angle, float offset_d) {
    float robot_w = ROBOT_WIDTH * ROBOT_SCALE;
    float robot_l = ROBOT_LENGTH * ROBOT_SCALE;

    float cos_a = cosf(angle);
    float sin_a = sinf(angle);

    // Calculate Robot Four Vertices
    POINT pts[4];
    float half_w = robot_w / 2.0f;
    float half_l = robot_l / 2.0f;

    float local_x[4] = {-half_w, half_w, half_w, -half_w};
    float local_y[4] = {half_l, half_l, -half_l, -half_l};

    for (int i = 0; i < 4; i++) {
        float rx = local_x[i] * cos_a - local_y[i] * sin_a;
        float ry = local_x[i] * sin_a + local_y[i] * cos_a;
        pts[i].x = ROBOT_CENTER_X + (int)rx;
        pts[i].y = ROBOT_CENTER_Y - (int)ry;
    }

    // Draw Robot Body
    setfillcolor(COLOR_ROBOT);
    setlinecolor(RGB(180, 180, 200));
    setlinestyle(PS_SOLID, 2);
    fillpolygon(pts, 4);

    // Draw Geometric Center (Red Dot)
    setfillcolor(COLOR_RED_CENTER);
    setlinecolor(COLOR_RED_CENTER);
    fillcircle(ROBOT_CENTER_X, ROBOT_CENTER_Y, 8);

    // Draw Actual Center (Blue Dot)
    float offset_px = offset_d * ROBOT_SCALE;
    float rx = offset_px * cos_a;
    float ry = offset_px * sin_a;
    int cx = ROBOT_CENTER_X + (int)rx;
    int cy = ROBOT_CENTER_Y - (int)ry;

    setfillcolor(COLOR_BLUE_CENTER);
    fillcircle(cx, cy, 8);

    // Blink if Offset Exceeds Threshold
    if (fabsf(offset_d) > OFFSET_THRESHOLD) {
        static int blink = 0;
        blink++;
        if ((blink / 5) % 2 == 0) {
            setfillcolor(RGB(255, 255, 0));
            fillcircle(cx, cy, 10);
        }
    }

    // Draw Legend
    int ly = 480;
    int lx = RIGHT_AREA_X + 20;
    settextstyle(18, 0, TEXT("Microsoft YaHei"));

    setfillcolor(COLOR_RED_CENTER);
    fillcircle(lx + 10, ly, 6);
    outtextxy(lx + 25, ly - 10, TEXT("Geometric Center (O)"));

    setfillcolor(COLOR_BLUE_CENTER);
    fillcircle(lx + 10, ly + 30, 6);
    outtextxy(lx + 25, ly + 20, TEXT("Actual Center (C)"));

    TCHAR txt[100];
    _stprintf_s(txt, TEXT("Current Offset: delta_d = %+.2f cm"), offset_d);
    outtextxy(lx, ly + 60, txt);
}

// Draw Speed Chart
void drawSpeedChart() {
    int cx = CHART_PADDING;
    int cy = 80;

    // Coordinate Axes
    setlinecolor(COLOR_TEXT);
    setlinestyle(PS_SOLID, 2);
    line(cx, cy + CHART_HEIGHT, cx + CHART_WIDTH, cy + CHART_HEIGHT);
    line(cx, cy, cx, cy + CHART_HEIGHT);

    // Grid
    setlinecolor(COLOR_GRID);
    setlinestyle(PS_DASH, 1);
    for (int i = 1; i <= 4; i++) {
        int gy = cy + i * CHART_HEIGHT / 4;
        line(cx, gy, cx + CHART_WIDTH, gy);
    }

    // Title
    settextcolor(COLOR_TEXT);
    settextstyle(18, 0, TEXT("Microsoft YaHei"));
    outtextxy(cx + 180, cy - 30, TEXT("Wheel Speed (cm/s)"));

    // Y-axis Scale
    settextstyle(14, 0, TEXT("SimSun"));
    TCHAR label[20];
    for (int i = 0; i <= 4; i++) {
        float val = 10.0f - i * 5.0f;
        int y = cy + i * CHART_HEIGHT / 4;
        _stprintf_s(label, TEXT("%.0f"), val);
        outtextxy(cx - 35, y - 7, label);
    }

    if (g_dataLog.empty()) {
        settextstyle(16, 0, TEXT("Microsoft YaHei"));
        settextcolor(RGB(120, 120, 120));
        outtextxy(cx + 80, cy + 120, TEXT("Press SPACE to Start"));
        return;
    }

    // Draw Curves
    size_t start = (g_dataLog.size() > MAX_DATA_POINTS) ?
                   g_dataLog.size() - MAX_DATA_POINTS : 0;
    float time_range = max(10.0f, g_dataLog.back().time);

    for (size_t i = start + 1; i < g_dataLog.size(); i++) {
        int x1 = cx + (int)((g_dataLog[i-1].time / time_range) * CHART_WIDTH);
        int x2 = cx + (int)((g_dataLog[i].time / time_range) * CHART_WIDTH);

        // V_L Green Line
        float vl1 = (10.0f - g_dataLog[i-1].V_L) / 20.0f;
        float vl2 = (10.0f - g_dataLog[i].V_L) / 20.0f;
        int y1_vl = cy + (int)(vl1 * CHART_HEIGHT);
        int y2_vl = cy + (int)(vl2 * CHART_HEIGHT);
        setlinecolor(COLOR_V_L);
        setlinestyle(PS_SOLID, 2);
        line(x1, y1_vl, x2, y2_vl);

        // V_R Blue Line
        float vr1 = (10.0f - g_dataLog[i-1].V_R) / 20.0f;
        float vr2 = (10.0f - g_dataLog[i].V_R) / 20.0f;
        int y1_vr = cy + (int)(vr1 * CHART_HEIGHT);
        int y2_vr = cy + (int)(vr2 * CHART_HEIGHT);
        setlinecolor(COLOR_V_R);
        line(x1, y1_vr, x2, y2_vr);
    }

    // Legend
    int lx = cx + 250;
    int ly = cy + 20;
    setlinecolor(COLOR_V_L);
    setlinestyle(PS_SOLID, 3);
    line(lx, ly, lx + 30, ly);
    settextcolor(COLOR_TEXT);
    settextstyle(14, 0, TEXT("Microsoft YaHei"));
    outtextxy(lx + 40, ly - 7, TEXT("V_L (Left)"));

    setlinecolor(COLOR_V_R);
    line(lx, ly + 25, lx + 30, ly + 25);
    outtextxy(lx + 40, ly + 18, TEXT("V_R (Right)"));
}

// Draw Offset Chart
void drawOffsetChart() {
    int cx = CHART_PADDING;
    int cy = 80 + CHART_HEIGHT + CHART_PADDING;

    // Coordinate Axes
    setlinecolor(COLOR_TEXT);
    setlinestyle(PS_SOLID, 2);
    line(cx, cy + CHART_HEIGHT, cx + CHART_WIDTH, cy + CHART_HEIGHT);
    line(cx, cy, cx, cy + CHART_HEIGHT);

    // Grid
    setlinecolor(COLOR_GRID);
    setlinestyle(PS_DASH, 1);
    for (int i = 1; i <= 4; i++) {
        int gy = cy + i * CHART_HEIGHT / 4;
        line(cx, gy, cx + CHART_WIDTH, gy);
    }

    // Success Criteria Line
    float norm_tol = (5.0f - SUCCESS_OFFSET_TOLERANCE) / 10.0f;
    int y1 = cy + (int)(norm_tol * CHART_HEIGHT);
    int y2 = cy + (int)((1.0f - norm_tol) * CHART_HEIGHT);
    setlinecolor(RGB(255, 200, 0));
    setlinestyle(PS_DOT, 1);
    line(cx, y1, cx + CHART_WIDTH, y1);
    line(cx, y2, cx + CHART_WIDTH, y2);

    // Title
    settextcolor(COLOR_TEXT);
    settextstyle(18, 0, TEXT("Microsoft YaHei"));
    outtextxy(cx + 180, cy - 30, TEXT("Offset (cm)"));

    // Y-axis Scale
    settextstyle(14, 0, TEXT("SimSun"));
    TCHAR label[20];
    for (int i = 0; i <= 4; i++) {
        float val = 5.0f - i * 2.5f;
        int y = cy + i * CHART_HEIGHT / 4;
        _stprintf_s(label, TEXT("%.1f"), val);
        outtextxy(cx - 35, y - 7, label);
    }

    if (g_dataLog.empty()) return;

    // Draw Curve
    size_t start = (g_dataLog.size() > MAX_DATA_POINTS) ?
                   g_dataLog.size() - MAX_DATA_POINTS : 0;
    float time_range = max(10.0f, g_dataLog.back().time);

    for (size_t i = start + 1; i < g_dataLog.size(); i++) {
        int x1 = cx + (int)((g_dataLog[i-1].time / time_range) * CHART_WIDTH);
        int x2 = cx + (int)((g_dataLog[i].time / time_range) * CHART_WIDTH);

        float od1 = (5.0f - g_dataLog[i-1].offset_d) / 10.0f;
        float od2 = (5.0f - g_dataLog[i].offset_d) / 10.0f;
        int y1 = cy + (int)(od1 * CHART_HEIGHT);
        int y2 = cy + (int)(od2 * CHART_HEIGHT);

        setlinecolor(COLOR_OFFSET);
        setlinestyle(PS_SOLID, 2);
        line(x1, y1, x2, y2);
    }

    // Legend
    int lx = cx + 220;
    int ly = cy + 20;
    setlinecolor(COLOR_OFFSET);
    setlinestyle(PS_SOLID, 3);
    line(lx, ly, lx + 30, ly);
    settextcolor(COLOR_TEXT);
    settextstyle(14, 0, TEXT("Microsoft YaHei"));
    outtextxy(lx + 40, ly - 7, TEXT("delta_d (Offset)"));

    setlinecolor(RGB(255, 200, 0));
    setlinestyle(PS_DOT, 2);
    line(lx, ly + 25, lx + 30, ly + 25);
    outtextxy(lx + 40, ly + 18, TEXT("Threshold (±1.5cm)"));
}

// Draw Info Panel
void drawInfoPanel() {
    int px = CHART_PADDING;
    int py = WINDOW_HEIGHT - 150;

    settextcolor(COLOR_TEXT);
    settextstyle(16, 0, TEXT("Microsoft YaHei"));

    TCHAR txt[200];
    _stprintf_s(txt, TEXT("Current Test: %s"), g_tests[g_currentTest].name);
    outtextxy(px, py, txt);

    settextstyle(14, 0, TEXT("Microsoft YaHei"));
    outtextxy(px, py + 25, g_tests[g_currentTest].desc);

    _stprintf_s(txt, TEXT("Simulation Time: %.2f s"), g_simTime);
    outtextxy(px, py + 55, txt);

    LPCTSTR status = g_isRunning ? (g_isPaused ? TEXT("Paused") : TEXT("Running")) : TEXT("Not Started");
    _stprintf_s(txt, TEXT("Status: %s"), status);
    outtextxy(px, py + 80, txt);

    if (!g_dataLog.empty()) {
        VisData& last = g_dataLog.back();
        _stprintf_s(txt, TEXT("V_L=%.2f  V_R=%.2f  delta_d=%+.3f cm"),
                   last.V_L, last.V_R, last.offset_d);
        outtextxy(px, py + 105, txt);
    }
}

// Draw Instructions
void drawInstructions() {
    int ix = RIGHT_AREA_X + 20;
    int iy = 540;

    settextcolor(RGB(160, 160, 160));
    settextstyle(14, 0, TEXT("Microsoft YaHei"));

    outtextxy(ix, iy, TEXT("Controls:"));
    outtextxy(ix, iy + 25, TEXT("SPACE - Start/Pause"));
    outtextxy(ix, iy + 45, TEXT("R - Reset"));
    outtextxy(ix, iy + 65, TEXT("N - Next Test"));
    outtextxy(ix, iy + 85, TEXT("P - Previous Test"));
    outtextxy(ix, iy + 105, TEXT("ESC/Q - Quit"));
}

// Run Simulation
void runSimulation() {
    static float current_offset = 0.0f;
    static float current_angle = 0.0f;

    if (g_dataLog.empty()) {
        current_offset = g_tests[g_currentTest].initial_offset;
        current_angle = 0.0f;
    }

    // Calculate Compensated Wheel Speeds
    float V_L = 0.0f;
    float V_R = 0.0f;
    compensate_speed(current_offset, &V_L, &V_R);

    // Update Offset
    float new_offset = calc_offset(V_L, V_R);
    current_offset = current_offset * 0.9f + new_offset * 0.1f;

    // Update Angle
    float omega = calc_angular_velocity(V_L, V_R);
    current_angle += omega * TIME_STEP;

    // Record Data
    addDataPoint(g_simTime, V_L, V_R, current_offset, current_angle);

    g_simTime += TIME_STEP;

    // Auto-pause after 30 seconds (manual pause with SPACE key anytime)
    if (g_simTime >= 30.0f) {
        g_isPaused = true;
    }
}

// Reset Simulation
void resetSimulation() {
    g_isRunning = false;
    g_isPaused = false;
    g_simTime = 0.0f;
    g_dataLog.clear();
}

// Add Data Point
void addDataPoint(float t, float vl, float vr, float od, float a) {
    VisData data;
    data.time = t;
    data.V_L = vl;
    data.V_R = vr;
    data.offset_d = od;
    data.angle = a;
    g_dataLog.push_back(data);
}
