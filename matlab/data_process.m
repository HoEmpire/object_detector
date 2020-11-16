data=csvread('data.csv',1,0);
data(:,[1,4]) = [];
index = [];
for i = 1:length(data)
    if abs(data(i,1)-data(i,3))>10
        index = [index i];
    end
end
data(index,:)=[];
data(:,4) = data(:,4)/180*pi;

theta_true = atan(data(:,3).*sin(data(:,4))./(data(:,3).*cos(data(:,4))-2.82))*180/pi;
range_true = data(:,3).*sin(data(:,4))./sin(theta_true/180*pi);
range_compare = [range_true, data(:,1)];
theta_delt = theta_true-data(:,2);

index = [];
for i = 1:length(theta_delt)
    if abs(theta_delt(i))>5
        index = [index i];
    end
end
theta_delt(index)=[];
theta_delt_ransac = ransac(theta_delt, 0.3,500);


mean(theta_delt_ransac)
std(theta_delt_ransac)

c= abs(range_compare(:,1)-range_compare(:,2));
index = [];
for i = 1:length(c)
    if abs(c(i))>5 | isnan(c(i))
        index = [index i];
    end
end
c(index) =[];
mean(c)
std(c)

function ransac_data_best = ransac(data, th,iter_num)
    eps=1e-6;
    ransac_count = 0;
    ransac_data_best = [];
    for iter=1:iter_num
        index = ceil(rand(1)*length(data));
        mean = data(index);
        ransac_data = [];
        for i = 1:length(data)
            if abs(data(i)-mean)/(abs(mean)+eps)<th
                ransac_data = [ransac_data data(i)];
            end
        end
        if length(ransac_data) > ransac_count
            ransac_count = ransac_data;
            ransac_data_best = ransac_data;
        end
    end
    ransac_data_best = ransac_data_best';
end

result = mean(ransac_data_best)