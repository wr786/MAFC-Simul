% author: wr786
% 默认target在(0, 0)

% =========================可编辑区====================================
% 用于设定机器人的初始位置与个数 
agentsPos = [ [1, sqrt(3)]; [-2, 2*sqrt(3)]; [-sqrt(3)/4, 1/4]; [-sqrt(3)/3, -1/3]; [-1, -sqrt(3)]; [1, -sqrt(3)] ]; 
% 用于设定目标队形机器人间相差的**角度制**角度，设intervals(i)=x(i%n + 1)-x(i)
intervals = [ 30; 90; 60; 30; 90; 60 ]; 
% 用于设定目标队形机器人离目标点的半径
targetRadius = 1;
% 用于设定期望转动角速度
omega = -0.3;
% ====================================================================

intervals = deg2rad(intervals); % 转为弧度制，方便计算

agentsPosX = agentsPos(:,1);
agentsPosY = agentsPos(:,2);
agentsNum = len(agentsPosX);    % 计算出用户设定的agents的总数
intervalsNum = len(intervals);  % 计算出给定的距离总数，数字应当相等于agents总数
assert(agentsNum == intervalsNum);    % 这两个数字应该相等

% 事实上agentsPosX和agentsPosY并不参与计算，这只是为了用户定位而设置的
% 实际运算中使用这两个：
agentsAngles = atan2(agentsPosY, agentsPosX);   % 即x，弧度制角度，初始相位
agentsRadius = get_dist(agentsPosY, agentsPosX);    % 初始半径
agentsBetas = -agentsAngles; % 初始beta

% 初始化
legends = string.empty;

% 作图
hold on
plot(0, 0, 'black.', "markerFaceColor", "black");    % 画出target坐标
legends(1) = "start";
plot(0, 0, 'k*');    % 画出target坐标
legends(2) = "end";
% 画出目标环
theta = linspace(0,2*pi);
x = targetRadius*cos(theta);
y = targetRadius*sin(theta);
plot(x, y, "k");
legends(3) = "target";
for i = 1: agentsNum % 标记初始化位置
    plot(agentsRadius(i) * cos(agentsAngles(i)), agentsRadius(i) * sin(agentsAngles(i)),'.', 'HandleVisibility','off', 'color', [i/agentsNum, 1 - i/agentsNum, 0.5 + i/2/agentsNum], 'markerFaceColor', [i/agentsNum, 1 - i/agentsNum, 0.5 + i/2/agentsNum]); % 画出起始坐标
    location(i) = plot(agentsRadius(i) * cos(agentsAngles(i)), agentsRadius(i) * sin(agentsAngles(i)),'*', 'HandleVisibility','off', 'color', [i/agentsNum, 1 - i/agentsNum, 0.5 + i/2/agentsNum]); % 坐标
    track(i) = plot(location(i).XData, location(i).YData, '-', 'color', [i/agentsNum, 1 - i/agentsNum, 0.5 + i/2/agentsNum]); % 軌跡
    v(i) = 0;   % 初速为0
    legends(i + 3) = sprintf('agent %d', i);
end
legend(legends);
axis equal;

dt = 0.1;  % Δt
aviobj = VideoWriter('output.avi','Motion JPEG AVI');
aviobj.Quality = 95;
open(aviobj);
for j = 1: 7867
    % 检查是否已经形成目标队形，若已形成则退出
    flag = 1;
    for i = 1: agentsNum
        % 检查距离
        if abs(agentsRadius(i) - targetRadius) > 1e-3
            flag = 0;
            break;
        end
        % 检查相位差
        if mod(abs(delta(agentsAngles, i) - intervals(i)), 2*pi) > 3e-2
            flag = 0;
            break;
        end
    end
    if flag == 1
        break;
    end
    % 更新agent位置状态
    for i = 1: agentsNum
        yi = delta(agentsAngles, i);    % yi
        yim = delta2(agentsAngles, i);   % yi-
        % 这里保证了yi与yim都是正的
        if yi < 0
            yi = yi + 2 * pi;
        end
        if yim < 0
            yim = yim + 2 * pi;
        end
        % 控制协议 ui
        ui = (intervals(minus_one(i, agentsNum)) * (yi) - intervals(i) * (yim)) / (intervals(i) + intervals(minus_one(i, agentsNum)));
        newOmega = omega + ui;  % 新角速度
        % newOmega = omega + selfish_factor(abs(agentsRadius(i) - targetRadius)) * ui;  % 加入自私因子的新角速度
        E = -( agentsRadius(i) - targetRadius ) / agentsRadius(i) - newOmega * (newOmega - 1); % 使用公式
        v(i) = (agentsRadius(i) * (E*cos(agentsBetas(i)) + newOmega * sin(agentsBetas(i))) - v(i)) * dt + v(i); % 计算新速度
        agentsRadius(i) = v(i) * cos(agentsBetas(i)) * dt + agentsRadius(i);   % 计算新半径
        % 计算新的beta
        agentsBetas(i)=(1 - agentsRadius(i) * (E*sin(agentsBetas(i)) - newOmega * cos(agentsBetas(i))) / v(i) - v(i) * sin(agentsBetas(i)) / agentsRadius(i)) * dt + agentsBetas(i);
        agentsAngles(i) = v(i) / agentsRadius(i) * sin(agentsBetas(i)) * dt + agentsAngles(i);   % 计算新相位
        % 画出结点位置
        location(i).XData = agentsRadius(i) * cos(agentsAngles(i));
        location(i).YData = agentsRadius(i) * sin(agentsAngles(i));
        track(i).XData = [track(i).XData, location(i).XData];
        track(i).YData = [track(i).YData, location(i).YData];
    end
    img = getframe(gcf);
    img.cdata = imresize(img.cdata, [768 1024]); 
    writeVideo(aviobj, img);
    drawnow;
end
hold off
close(aviobj);
disp("[INFO] AOK");
% ===================================函数区===============================

function ret = selfish_factor(dist)
    ret = min(0.3 / dist, 1);
end

% 创建函数方便计算
function l = len(Array)
    tmp = size(Array);
    l = tmp(1);
end

% 这个函数又可以计算到target的距离又可以用来计算相对距离
function ret = get_dist(x, y)
    ret = sqrt(x.^2 + y.^2);
end

function ret = plus_one(x, modNum)
    ret = mod(x, modNum) + 1;
end

function ret = minus_one(x, modNum)
    ret = mod(x + modNum - 2, modNum) + 1;
end

function ret = delta(Array, i)
    ret = Array(plus_one(i, len(Array))) - Array(i);
end

function ret = delta2(Array, i)
    ret = Array(i) - Array(minus_one(i, len(Array)));
end