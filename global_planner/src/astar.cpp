/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include<global_planner/astar.h>
#include<costmap_2d/cost_values.h>

namespace global_planner {

AStarExpansion::AStarExpansion(PotentialCalculator* p_calc, int xs, int ys) :
        Expander(p_calc, xs, ys) {
}
//A*算法的实现
bool AStarExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) {
    queue_.clear();
    int start_i = toIndex(start_x, start_y);//将初始坐标转换成地图栅格的序号
    queue_.push_back(Index(start_i, 0));//把初始点的potential值设为0，
                                        //index类：第一个代表栅格地图的序号，第二个代表f, cost
    std::fill(potential, potential + ns_, POT_HIGH);//所有位置的f都设置为最大1e10
    potential[start_i] = 0;//potential就是估计值g,f=g+h

    int goal_i = toIndex(end_x, end_y);//将终点坐标转换成地图栅格的序号
    int cycle = 0;
    //????????????????????cycles=nx*ny*2
    while (queue_.size() > 0 && cycle < cycles) {
        Index top = queue_[0];//get the Index that with mini cost
        std::pop_heap(queue_.begin(), queue_.end(), greater1());//把首元素放到最后，其他元素按照cost值从小到大排列
        queue_.pop_back();//删除最后一个元素with mini cost//remove the Index with mini cost

        int i = top.i;
        if (i == goal_i)//计算到goal就结束
            return true;
        //add the neighborhood 4 points into the search scope
        add(costs, potential, potential[i], i + 1, end_x, end_y);
        add(costs, potential, potential[i], i - 1, end_x, end_y);
        add(costs, potential, potential[i], i + nx_, end_x, end_y);//因为数组上下相差一个nx
        add(costs, potential, potential[i], i - nx_, end_x, end_y);

        cycle++;
    }

    return false;
}

void AStarExpansion::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x,
                         int end_y) {
    if (next_i < 0 || next_i >= ns_)//ns_为栅格的总的数目
        return;

    if (potential[next_i] < POT_HIGH)//没有搜索过的元素r都是最大值，不是则在close列表里面，不再搜索
        return;
//    static const unsigned char NO_INFORMATION = 255;
//    static const unsigned char LETHAL_OBSTACLE = 254;
//    static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
//    static const unsigned char FREE_SPACE = 0;
    //it means this cell is obstacle lethal cost is 253  //neutral_cost_设置默认值是50
    if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))
        return;

    // compute the next_i cell in potential
    //petential存储所有点的g(n)实在状态空间中从初始节点到n节点的实际代价
    //costs[next_i] + neutral_cost_, next_i,这个点在costmap上的代价值，加neutral-cost，是因为每一次远离起点一个栅格，
    //prev_potential,当前点的f
    //calculatePotential根据use_quadratic的不同有两个选择
    //默认true,quadratioCalculator二次曲线计算
    //false简单的计算virtual float calculatePotential: return potential[next-i]=pre_potential+costs[next_i] + neutral_cost_
    potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);//核心

    int x = next_i % nx_, y = next_i / nx_;
    //distance * neutral_cost_ means the current to the target
    float distance = abs(end_x - x) + abs(end_y - y);//代价h（n）
    //A*的核心思想
    queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));//f=potential[next_i] + distance * neutral_cost_
    std::push_heap(queue_.begin(), queue_.end(), greater1());//对刚插入的尾部元素做堆排序从小到大
}

} //end namespace global_planner
