# ------------------------------------------------------------------------------------------------
# Copyright (c) 2018 Microsoft Corporation
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
# associated documentation files (the "Software"), to deal in the Software without restriction,
# including without limitation the rights to use, copy, modify, merge, publish, distribute,
# sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all copies or
# substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
# NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
# DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
# ------------------------------------------------------------------------------------------------

import malmoenv
import argparse
from pathlib import Path
import time
from PIL import Image

# 加入 LLM

# 加入 memory




if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='malmovnv test')
    parser.add_argument('--mission', type=str, default='simulator/MalmoEnv/missions/mobchase_single_agent.xml', help='the mission xml')
    parser.add_argument('--port', type=int, default=9000, help='the mission server port')
    parser.add_argument('--server', type=str, default='127.0.0.1', help='the mission server DNS or IP address')
    parser.add_argument('--port2', type=int, default=None, help="(Multi-agent) role N's mission port. Defaults to server port.")
    parser.add_argument('--server2', type=str, default=None, help="(Multi-agent) role N's server DNS or IP")
    parser.add_argument('--episodes', type=int, default=1, help='the number of resets to perform - default is 1')
    parser.add_argument('--episode', type=int, default=0, help='the start episode - default is 0')
    parser.add_argument('--role', type=int, default=0, help='the agent role - defaults to 0')
    parser.add_argument('--episodemaxsteps', type=int, default=0, help='max number of steps per episode')
    parser.add_argument('--saveimagesteps', type=int, default=0, help='save an image every N steps')
    parser.add_argument('--resync', type=int, default=0, help='exit and re-sync every N resets'
                                                              ' - default is 0 meaning never.')
    parser.add_argument('--experimentUniqueId', type=str, default='test1', help="the experiment's unique id.")
    args = parser.parse_args()
    if args.server2 is None:
        args.server2 = args.server

    xml = Path(args.mission).read_text()
    env = malmoenv.make()
    
    # 添加 动作过滤器 所有动作都在这个范围内
    # action_filter = {"move", "turn", "use", "attack"}
    action_filter = {"move", "turn", "use", "attack", "look", "jump"}

    # 将xml中内容传入并接卸
    env.init(xml, args.port,
             server=args.server,
             server2=args.server2, port2=args.port2,
             role=args.role,
             exp_uid=args.experimentUniqueId,
             episode=args.episode,
             action_filter=action_filter,
             resync=args.resync)

    # 在当前目录下创建log文件夹，并获取当前时间作为log文件名
    log_dir = Path('log')
    log_dir.mkdir(exist_ok=True)
    log_file = log_dir / f'action_{time.strftime("%Y%m%d_%H%M%S")}.log'

    # 清空action.log写入实验信息
    with open(log_file, 'a') as f:
        f.write('======================\n')
        f.write('======================\n')
        f.write('xml ' + xml + '\n')
        f.write('======================\n')
        f.write('======================\n')
        
        
        f.write('mission ' + args.mission + '\n')
        f.write('role ' + str(args.role) + '\n')
        f.write('exp_uid ' + args.experimentUniqueId + '\n')
        f.write('episodes ' + str(args.episodes) + '\n')
        f.write('episode ' + str(args.episode) + '\n')
        f.write('resync ' + str(args.resync) + '\n')
        f.write('======================\n')

        f.write('env.commands ' + str(env.commands) + '\n')
        f.write('env.actions ' + str(env.actions) + '\n')
        # 写入多行回车
        f.write('\n\n\n\n')
        
    
    # # 删除 log 文件
    # if log_file.exists():
    #     log_file.unlink()
    #     print(f"Deleted existing log file: {log_file}")
    # # 调试退出
    # exit(0)

    for i in range(args.episodes):
        print("reset " + str(i))
        obs = env.reset()
        
        # 打开action.log将写入 episode i
        with open(log_file, 'a') as f:
            f.write('\n\n\n\nepisode ' + str(i) + '\n')
            f.write('======================\n')
            
            
        # ADD : 获取用户指令
        user_request = ''
        # user_request = input("Press Enter to continue, or type 'exit' to quit: ")
        if user_request.lower() == 'exit':
            print("Exiting the experiment.")
            break
        else:
            print("Continuing the experiment.")
            
            
        # add : 根据 env.actions 建立 Graph
        

        steps = 0
        done = False
        while not done and (args.episodemaxsteps <= 0 or steps < args.episodemaxsteps):
            # ADD:获取当前环境信息并更新 - 聚类，将环境中能聚在一起的物体整合成一个 box 迁出一个索引
            env.render()
            
            # 这里是随机指令
            # action = env.action_space.sample()
            
            # add 根据当前环境和用户指令生成一系列动作
            action_sequence = []

                
            # 调试 根据用户输入数字进行相应的操作
            # 打印索引并询问用户输入索引
            print("Available actions:")
            
            # 每5个换行
            for i, act in enumerate(env.action_space):
                print(f"{i}: {act}", end='\t')
                if (i + 1) % 5 == 0:
                    print()

            user_input = input("q")
            if user_input.lower() == 'q':
                break
            if not user_input.isdigit() or int(user_input) < 0 or int(user_input) >= len(env.actions):
                action = env.action_space.sample()
            else:
                action = int(user_input)
                
            
                
            # 清理终端
            print("\033c", end="")  # ANSI escape code to clear the terminal
                
            # 打开action.log将当前指令写入文件
            with open(log_file, 'a') as f:
                f.write("action: " + str(action) + ',' + env.action_space[action] + '\n')
                
            print(action , str(type(action)))
            
            obs, reward, done, info = env.step(action)
            steps += 1
            
            # 将以上信息写入action.log
            with open(log_file, 'a') as f:
                f.write('reward: ' + str(reward) + '\n')
                f.write('done: ' + str(done) + '\n')
                f.write('obs: ' + str(obs) + '\n')
                f.write('info: ' + info + '\n')
                f.write('-------------------------\n')
                
                
            print("action: " + str(action) + ',' + env.action_space[action])
            print("reward: " + str(reward))
            print("done: " + str(done))
            print("obs: " + str(obs))
            print("info" + info)
            # 将 info 字符串 转成 info 字典
            # info = eval(info) if info else {}
            # 打印出 info 字典的 board 信息
            # print("info board: " + str(info.get('board', 'N/A')))
            
            
            
            
            
            
            
            
            
            
            if args.saveimagesteps > 0 and steps % args.saveimagesteps == 0:
                h, w, d = env.observation_space.shape
                img = Image.fromarray(obs.reshape(h, w, d))
                img.save('image' + str(args.role) + '_' + str(steps) + '.png')

            time.sleep(.05)

    env.close()
