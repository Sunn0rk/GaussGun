% Создание массива, для построения графика
Max = 130;
Lstep = 10;
Sila = zeros(Max,Lstep);
Inductance = zeros(Max,Lstep);
% Начало цикла
for v = 1:Max
    for V = 1:Lstep
        % Начальные данные
        r = 5/1000;
        mu = 2270;
        mu0 = 4*pi*10^-7;
        S0 = 60/1000;
        H0 = v/1000;
        I = 0.670;
        R = 8.5/1000;
        L = 57/1000;
        w = 2400;
        i = fix(w);
        j = 50;
        l = L;
%         S = 2*pi*r*l;
        S = pi*r^2;
        % Создание массивов
        Si = zeros(i,1);
        Hj = zeros(j,1);
        Bi = zeros(i,1);
        Bj = zeros(j,1);
        
        % Установка расположения катушки и снаряда
        for k = 1:i
            Si(k) = S0 + (k-1) * (L/i) + (L/i) / 2;
        end
        for K = 1:j
            Hj(K) = H0 + (K-1) * (l/j) + (l/j) / 2;
        end
        % Расчет магнитной индукции и силы втягивания
        for K = 1:j
            for k = 1:i
              h = Si(k)-Hj(K);
              Bi(k) = abs(((mu * mu0 * I * R * h) / (2 * ((R)^2 + (h^2))^1.5))*w/i/2);
            end
            Bj(K) = sum(Bi)/j;
        end
        B = sum(Bj);
            Inductance(v,V) = B;
            ABS = B/abs(B);
            F = B^2*S/(2*mu*mu0)*ABS;
        % Запись вычисленной силы в массив
        Sila((v),V) = F;
    end
end
% Отрисовка графика зависимости силы от расстояния
subplot(2,1,1)
plot(Sila);
subplot(2,1,2)
plot(Inductance);
