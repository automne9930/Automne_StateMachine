/**
 * @file StateMachine.h
 * @brief 表驱动式状态机 - 详细注释版
 * 
 * 【什么是表驱动状态机】
 * 把"状态A + 事件X → 状态B"这种跳转规则存成一张表，
 * 触发事件时查表决定跳到哪个状态，而不是写一堆if-else。
 * 
 * 【为什么用它】
 * 1. 加新状态只需加几行规则，不用改其他代码
 * 2. 转移规则集中管理，一目了然
 * 3. 支持进入/退出/更新回调，逻辑分离
 */

#pragma once

#include <functional>
#include <unordered_map>
#include <vector>

/**
 * @brief 状态机模板类
 * @tparam StateT 状态类型（通常是enum）
 * @tparam EventT 事件类型（通常是enum）
 * 
 * 使用示例:
 * ```cpp
 * // 定义状态和事件
 * enum State { IDLE, STAND, WALK };
 * enum Event { EVT_START, EVT_STOP };
 * 
 * // 创建状态机，初始状态IDLE
 * StateMachine<State, Event> fsm(IDLE);
 * 
 * // 添加转移规则：当前状态 + 事件 → 目标状态
 * fsm.addTransition(IDLE, EVT_START, STAND);
 * fsm.addTransition(STAND, EVT_START, WALK);
 * 
 * // 触发事件
 * fsm.trigger(EVT_START);  // IDLE → STAND
 * fsm.trigger(EVT_START);  // STAND → WALK
 * ```
 */
template <typename StateT, typename EventT>
class StateMachine {
public:
    //==========================================================
    // 类型定义 - 各种回调函数的类型
    //==========================================================
    
    // 进入状态时调用的函数，无参数
    using EnterCallback = std::function<void()>;
    
    // 退出状态时调用的函数，无参数
    using ExitCallback = std::function<void()>;
    
    // 每帧更新时调用的函数，参数是时间步长dt
    using UpdateCallback = std::function<void(double dt)>;
    
    // 条件守卫函数，返回true才允许跳转
    using GuardFunc = std::function<bool()>;

    //==========================================================
    // 转移规则结构体 - 表里的每一行
    //==========================================================
    
    /**
     * 一条转移规则，表示：
     * "当处于from状态时，如果收到event事件，且guard条件满足，就跳到to状态"
     */
    struct Transition {
        StateT from;      // 源状态
        EventT event;     // 触发事件
        StateT to;        // 目标状态
        GuardFunc guard;  // 条件守卫（可选），为nullptr表示无条件
    };

private:
    //==========================================================
    // 成员变量
    //==========================================================
    
    StateT currentState_;   // 当前状态
    StateT prevState_;      // 上一个状态（用于查询"从哪来的"）
    double stateTime_ = 0;  // 在当前状态已经待了多久（秒）
    
    // 转移规则表：存储所有的跳转规则
    std::vector<Transition> transitions_;
    
    // 回调函数表：key是状态（转成int），value是回调函数
    std::unordered_map<int, EnterCallback> enterCallbacks_;   // 进入回调
    std::unordered_map<int, ExitCallback> exitCallbacks_;     // 退出回调
    std::unordered_map<int, UpdateCallback> updateCallbacks_; // 更新回调
    
    // 超时转移表：在某状态待够一定时间后自动跳转
    struct TimeoutTransition {
        StateT from;     // 源状态
        StateT to;       // 目标状态
        double timeout;  // 超时时间（秒）
    };
    std::vector<TimeoutTransition> timeoutTransitions_;

public:
    //==========================================================
    // 构造函数
    //==========================================================
    
    /**
     * @brief 创建状态机
     * @param initState 初始状态
     */
    explicit StateMachine(StateT initState) 
        : currentState_(initState), prevState_(initState) {}

    //==========================================================
    // 配置接口 - 用于设置转移规则和回调
    //==========================================================
    
    /**
     * @brief 添加一条转移规则
     * @param from  源状态 - 当前必须处于这个状态
     * @param event 触发事件 - 收到这个事件时检查
     * @param to    目标状态 - 条件满足就跳到这里
     * @param guard 条件守卫 - 可选，返回true才跳转
     * 
     * 示例:
     * ```cpp
     * // 无条件跳转：IDLE收到START就跳到STAND
     * fsm.addTransition(IDLE, EVT_START, STAND);
     * 
     * // 有条件跳转：WALK收到RUN且速度>2才跳到RUN状态
     * fsm.addTransition(WALK, EVT_RUN, RUN, [](){ return speed > 2; });
     * ```
     */
    void addTransition(StateT from, EventT event, StateT to, 
                       GuardFunc guard = nullptr) {
        transitions_.push_back({from, event, to, guard});
    }

    /**
     * @brief 添加超时自动转移
     * @param from    源状态
     * @param to      目标状态
     * @param timeout 超时时间（秒）
     * 
     * 示例:
     * ```cpp
     * // 在INIT状态待满2秒后自动跳到STAND
     * fsm.addTimeoutTransition(INIT, STAND, 2.0);
     * ```
     */
    void addTimeoutTransition(StateT from, StateT to, double timeout) {
        timeoutTransitions_.push_back({from, to, timeout});
    }

    /**
     * @brief 设置进入某状态时的回调
     * @param state 状态
     * @param cb    回调函数，刚进入该状态时调用一次
     * 
     * 示例:
     * ```cpp
     * fsm.onEnter(STAND, []() {
     *     printf("进入站立状态\n");
     *     initStandPose();
     * });
     * ```
     */
    void onEnter(StateT state, EnterCallback cb) {
        enterCallbacks_[static_cast<int>(state)] = cb;
    }

    /**
     * @brief 设置退出某状态时的回调
     * @param state 状态
     * @param cb    回调函数，即将离开该状态时调用一次
     */
    void onExit(StateT state, ExitCallback cb) {
        exitCallbacks_[static_cast<int>(state)] = cb;
    }

    /**
     * @brief 设置某状态的每帧更新回调
     * @param state 状态
     * @param cb    回调函数，处于该状态时每帧调用
     * 
     * 示例:
     * ```cpp
     * fsm.onUpdate(WALK, [](double dt) {
     *     // 每帧执行行走控制
     *     walkController(dt);
     * });
     * ```
     */
    void onUpdate(StateT state, UpdateCallback cb) {
        updateCallbacks_[static_cast<int>(state)] = cb;
    }

    //==========================================================
    // 运行接口 - 用于驱动状态机运行
    //==========================================================

    /**
     * @brief 触发事件，尝试状态转移
     * @param event 事件
     * @return true 如果发生了状态转移
     * 
     * 工作流程:
     * 1. 遍历转移规则表
     * 2. 找到 from==当前状态 且 event==触发事件 的规则
     * 3. 如果有guard条件，检查是否满足
     * 4. 条件满足则执行跳转
     */
    bool trigger(EventT event) {
        // 遍历所有转移规则
        for (const auto& t : transitions_) {
            // 检查：当前状态匹配 且 事件匹配
            if (t.from == currentState_ && t.event == event) {
                // 检查条件守卫（如果有的话）
                if (t.guard == nullptr || t.guard()) {
                    // 执行状态跳转
                    doTransition(t.to);
                    return true;
                }
            }
        }
        return false;  // 没找到匹配的规则，不跳转
    }

    /**
     * @brief 强制切换状态（忽略转移规则）
     * @param state 目标状态
     * 
     * 用于特殊情况，比如错误恢复、紧急停止等
     */
    void forceState(StateT state) {
        doTransition(state);
    }

    /**
     * @brief 更新状态机（每帧调用）
     * @param dt 时间步长（秒）
     * 
     * 工作流程:
     * 1. 累加状态持续时间
     * 2. 检查是否有超时转移需要执行
     * 3. 调用当前状态的update回调
     */
    void update(double dt) {
        // 累加在当前状态的时间
        stateTime_ += dt;
        
        // 检查超时转移
        for (const auto& t : timeoutTransitions_) {
            if (t.from == currentState_ && stateTime_ >= t.timeout) {
                doTransition(t.to);
                return;  // 跳转后直接返回，不执行update
            }
        }
        
        // 执行当前状态的update回调
        auto it = updateCallbacks_.find(static_cast<int>(currentState_));
        if (it != updateCallbacks_.end() && it->second) {
            it->second(dt);
        }
    }

    //==========================================================
    // 查询接口 - 用于获取状态机信息
    //==========================================================

    /** @brief 获取当前状态 */
    StateT getState() const { return currentState_; }
    
    /** @brief 获取上一个状态 */
    StateT getPrevState() const { return prevState_; }
    
    /** @brief 获取在当前状态已持续的时间（秒） */
    double getStateTime() const { return stateTime_; }
    
    /** @brief 检查是否处于某状态 */
    bool isState(StateT state) const { return currentState_ == state; }
    
    /** @brief 检查是否刚刚进入当前状态（stateTime==0） */
    bool justEntered() const { return stateTime_ == 0; }

private:
    //==========================================================
    // 内部函数
    //==========================================================
    
    /**
     * @brief 执行状态跳转
     * @param newState 目标状态
     * 
     * 执行顺序:
     * 1. 调用旧状态的退出回调
     * 2. 更新状态变量
     * 3. 重置状态计时器
     * 4. 调用新状态的进入回调
     */
    void doTransition(StateT newState) {
        // 如果目标状态和当前状态相同，不做任何事
        if (newState == currentState_) return;
        
        // 1. 调用旧状态的退出回调
        auto exitIt = exitCallbacks_.find(static_cast<int>(currentState_));
        if (exitIt != exitCallbacks_.end() && exitIt->second) {
            exitIt->second();
        }
        
        // 2. 更新状态
        prevState_ = currentState_;
        currentState_ = newState;
        stateTime_ = 0;  // 重置计时器
        
        // 3. 调用新状态的进入回调
        auto enterIt = enterCallbacks_.find(static_cast<int>(currentState_));
        if (enterIt != enterCallbacks_.end() && enterIt->second) {
            enterIt->second();
        }
    }
};
