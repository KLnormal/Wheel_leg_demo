len = 0.05:0.01:0.15;
N = numel(len);
K_matrix_all = cell(1,N);
for i = 1:N
    K_matrix_all{i} = get_k_lenth(len(i));
end
Ary = zeros(2,6,N);
for i = 1:N 
   for k = 1:6
       for l = 1:2
           Ary(l,k,i) = K_matrix_all{i}(l,k);
       end
   end
end        
deg = 4;
data = zeros(2,6,deg+1);
for i = 1:2
    for j = 1:6
        y = squeeze(Ary(i,j,:));
        x = len;
        p = polyfit(x,y,deg);
        data(i,j,:) = p; 
    end
end
figure; 
for i = 1:2
    for j = 1:6
        idx = (i-1)*6 + j;    % 子图编号 1~12
        subplot(2,6,idx);
        y = squeeze(Ary(i,j,:));
        % for k = 0.05:0.01:0.15
        %     temp = 0;
        %     for cnt = 0:deg
        %         temp = temp+ data(i,j,cnt+1)*(k^(deg-cnt));
        %     end
        %     subplot(k,temp,'ro',idx);
        % end
        temp_p = squeeze(data(i,j,:));
        y_fit = polyval(temp_p, len);
        plot(len, y, '-o', 'LineWidth', 1.2);
        hold on;
        plot(len, y_fit, '-r', 'LineWidth', 1.5);
        hold off;
        xlabel('Length'); ylabel(sprintf('K(%d,%d)', i,j));
        grid on;
    end
end

[nRows, nCols, deg_plus1] = size(data);  % 2×6×5
deg = deg_plus1 - 1;

for i = 1:nRows
    for j = 1:nCols
        coeffs = squeeze(data(i,j,:));  % [a1 a2 a3 a4 a5]
        
        % 构造公式字符串
        str = sprintf('K%d%d = ', i, j);
        for k = 1:length(coeffs)
            power = deg - (k-1);
            a = coeffs(k);
            if k < length(coeffs)
                str = [str, sprintf('(%g)*pow(len,%d) + ', a, power)];
            else
                str = [str, sprintf('(%g)', a)];
            end
        end
        disp(str);
    end
end
disp("finish");