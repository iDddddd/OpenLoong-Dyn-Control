/*
This is part of OpenLoong Dynamics Control, an open project for the control of biped robot,
Copyright (C) 2024-2025 Humanoid Robot (Shanghai) Co., Ltd.
Feel free to use in any purpose, and cite OpenLoong-Dynamics-Control in any style, to contribute to the advancement of the community.
 <https://atomgit.com/openloong/openloong-dyn-control.git>
 <web@openloong.org.cn>
*/
#include "GLFW_callbacks.h"

UIctr::UIctr(mjModel *modelIn, mjData *dataIn) {
    mj_model=modelIn;
    mj_data=dataIn;
    cam=mjvCamera();
    opt=mjvOption();
    scn=mjvScene();
    con=mjrContext();
}
// 声明外部力矩变量
extern std::vector<double> g_motor_torques;

extern std::vector<std::string> g_joint_names;



static void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    ((UIctr*)(glfwGetWindowUserPointer(window)))->Scroll(xoffset, yoffset);
}
static void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    ((UIctr*)(glfwGetWindowUserPointer(window)))->Mouse_move(xpos, ypos);
}
static void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    ((UIctr*)(glfwGetWindowUserPointer(window)))->Mouse_button(button, act, mods);
}

static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    ((UIctr*)(glfwGetWindowUserPointer(window)))->Keyboard(key, scancode, act, mods);
}

static void window_close_callback(GLFWwindow* window)
{
    ((UIctr*)(glfwGetWindowUserPointer(window)))->Close();
}

void UIctr::iniGLFW() {
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");
//    char **tmp;
//    glutInit(0,tmp);
    //glutDisplayFunc(UIctr::displaySimTime);
}

// create window, make OpenGL context current, request v-sync, adjust view, bond callbacks, etc.
void UIctr::createWindow(const char* windowTitle, bool saveVideo) {
    window=glfwCreateWindow(width, height, windowTitle, NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    mjv_defaultCamera(&cam);
    // Set up mujoco visualization objects
    // adjust view point
    double arr_view[] = {150, -16, 3, 0, 0.000000, 1.00000}; //view the right side
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];

    if (isTrack) {
        cam.lookat[2] += 0.8;
        cam.type = mjCAMERA_TRACKING;
        cam.trackbodyid = 1;
    }
    else
        cam.type = mjCAMERA_FREE;

    //mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(mj_model, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(mj_model, &con, mjFONTSCALE_150);   // model-specific context
    mjv_moveCamera(mj_model, mjMOUSE_ROTATE_H, 0.0, 0.0, &scn, &cam);

    // install GLFW mouse and keyboard callbacks
    glfwSetWindowUserPointer(window, this);
    glfwSetWindowCloseCallback(window, window_close_callback);
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    save_video=saveVideo;
    if (save_video)
    {
        image_rgb_ = (unsigned char*)malloc(3*width*height*sizeof(unsigned char));
        image_depth_ = (float*)malloc(sizeof(float)*width*height);

        // create output rgb file
        file = fopen("../record/rgbRec.out", "wb");
        if( !file )
            mju_error("Could not open rgbfile for writing");
    }
}
void UIctr::displayJointTorques() {
    // 检查力矩数据是否有效
    if (g_motor_torques.empty()) {
        return;
    }
    
    // 获取视口信息
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    
    // 定义要显示的腿部关节索引和名称
    std::vector<int> leg_joint_indices = {
        19, 20, 21, 22, 23, 24, // 左腿关节索引
        25, 26, 27, 28, 29, 30  // 右腿关节索引
    };
    
    std::vector<std::string> leg_joint_names = {
        "L_Hip_Roll", "L_Hip_Yaw", "L_Hip_Pitch", "L_Knee", "L_Ankle_Pitch", "L_Ankle_Roll",
        "R_Hip_Roll", "R_Hip_Yaw", "R_Hip_Pitch", "R_Knee", "R_Ankle_Pitch", "R_Ankle_Roll"
    };
    
    // 创建一个足够大的buffer来存储所有文本
    char buffer[1024];  // 增加buffer大小以容纳所有行
    int offset = 0;
    
    // 添加标题
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "=== joint torques (Nm) ===\n");
    
    // 添加所有关节力矩信息
    for (size_t i = 0; i < leg_joint_indices.size() && i < leg_joint_names.size(); ++i) {
        int idx = leg_joint_indices[i];
        if (idx >= 0 && idx < (int)g_motor_torques.size()) {
            // 格式化输出，确保对齐
            offset += snprintf(buffer + offset, sizeof(buffer) - offset, 
                              "%-18s: %6.2f\n", 
                              leg_joint_names[i].c_str(), g_motor_torques[idx]);
        }
    }
    
    // 一次性显示所有文本
    mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMRIGHT, viewport, buffer, NULL, &con);
}


void UIctr::updateScene() {
    if (!isContinuous)
        runSim= false;

    buttonRead.key_w=false;
    buttonRead.key_a=false;
    buttonRead.key_s=false;
    buttonRead.key_d=false;
    buttonRead.key_space=false;
    buttonRead.key_h=false;
    buttonRead.key_j=false;

    // 获取帧缓冲区视口
    mjrRect viewport = {0, 0, 0, 0};
    glfwMakeContextCurrent(window);
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // 更新场景并渲染
    mjv_updateScene(mj_model, mj_data, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);
    
    // 所有的overlay渲染必须在mjr_render之后、glfwSwapBuffers之前调用
    
    // 显示时间信息
    char buffer[100];
    std::sprintf(buffer, "Time: %.3f", mj_data->time);
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, viewport, buffer, NULL, &con);
    
    // 显示关节力矩
    displayJointTorques();

    // 保存视频帧（如果启用）- 必须在所有overlay渲染之后、缓冲区交换之前
    if (save_video)
    {
        mjr_readPixels(image_rgb_, image_depth_, viewport, &con);
        fwrite(image_rgb_, sizeof(unsigned char), 3*width*height, file);
    }
    
    // 现在再交换缓冲区
    glfwSwapBuffers(window);
    // 处理GUI事件
    glfwPollEvents();
}



// keyboard callback
void UIctr::Keyboard(int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(mj_model, mj_data);
        mj_forward(mj_model, mj_data);
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_1)
    {
        runSim=!runSim;
        isContinuous= true;
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_2)
    {
        runSim= true;
        isContinuous= false;
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_W){
        buttonRead.key_w= true;
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_A){
        buttonRead.key_a= true;
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_S){
        buttonRead.key_s= true;
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_D){
        buttonRead.key_d= true;
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_H){
        buttonRead.key_h= true;
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_J){
        buttonRead.key_j= true;
    }

    if (act==GLFW_RELEASE && key==GLFW_KEY_SPACE){
        buttonRead.key_space= true;
    }
}

// mouse button callback
void UIctr::Mouse_button(int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void UIctr::Mouse_move(double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(mj_model, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void UIctr::Scroll(double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(mj_model, mjMOUSE_ZOOM, 0, 0.05*yoffset, &scn, &cam);
}

void UIctr::Close() {
    // Free mujoco objects
    mj_deleteData(mj_data);
    mj_deleteModel(mj_model);
    mjr_freeContext(&con);
    mjv_freeScene(&scn);


    glfwTerminate();
}

void UIctr::enableTracking() {
    isTrack=true;
}

UIctr::ButtonState UIctr::getButtonState() {
    ButtonState tmp=buttonRead;
    buttonRead.key_w= false;
    buttonRead.key_a= false;
    buttonRead.key_s= false;
    buttonRead.key_d= false;
    buttonRead.key_h= false;
    buttonRead.key_j= false;
    buttonRead.key_space= false;
    return tmp;
}