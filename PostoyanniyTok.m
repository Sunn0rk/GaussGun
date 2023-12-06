    clc;clear;
    % Начальные данные
    while 1

        % Параметры источника питания
        Toque = 3;

        % Массивы параметров катушек
        CounterCoil = 10;
        InductorLength = zeros(CounterCoil+1,1);
        InductorRadius = zeros(CounterCoil+1,1);
        QuantityCoil = zeros(CounterCoil+1,1);
        InductorStartPosition = zeros(CounterCoil+1,1);
        Inductance = zeros(CounterCoil+1,1);
        % Параметры катушек
        for a = 1:CounterCoil+1
            InductorLength(a) = 50/1000;
            InductorRadius(a) = 16/1000;
            QuantityCoil(a) = 4*InductorLength(a)*1000;
            InductorStartPosition(a) = (20+(a-1)*(InductorLength(a)*1000+30))/1000;
            Inductance(a) = 633e-6;
        end
        % Мнимая послепоследняя катушка
        InductorStartPosition(CounterCoil+1) = 1000;

        % парметры снаряда
        BulletRadius = 15/1000;
        mu = 120;
        BulletLength = InductorLength(1)*3/4;
        StartCoordinates = InductorStartPosition(1)-InductorLength(1)*0.4;
        Plotnost = 7800;
        Weight = pi*BulletLength*BulletRadius^2*Plotnost;
        MaxB = 2.12;

        % Остальные
        mu0 = 4*pi*10^-7;
        CoilAccuracy = fix(QuantityCoil(a));
        BulletAccuracy = 50;
        StartSpeed = 0;
        StartTime = 1;
        break
    end
    
    % Создание массивов
    while 1
        ToqueDischargeTimeArray = linspace(0, 0.3, 600);
        InductorLocation = zeros(CoilAccuracy,CounterCoil);
        BulletLocation = zeros(BulletAccuracy,1);
        H_from_Coil = zeros(CoilAccuracy,1);
        H_from_Bullet = zeros(BulletAccuracy,1);
        AccelerationArray = zeros(length(ToqueDischargeTimeArray),1);
        SpeedArray = zeros(length(ToqueDischargeTimeArray),1);
        CoordinateArray = zeros(length(ToqueDischargeTimeArray),1);
        TimeStepArray = zeros(length(ToqueDischargeTimeArray),1);
        SupportArray = zeros(length(ToqueDischargeTimeArray),1);
        break
    end

    % Установка расположения катушек и снаряда
    while 1

        for a = 1:CounterCoil
            for b = 1:CoilAccuracy
                InductorLocation(b,a) = InductorStartPosition(a) + (b-1) * (InductorLength(a)/CoilAccuracy) + (InductorLength(a)/CoilAccuracy) / 2;
            end
        end

        for b = 1:BulletAccuracy
            BulletLocation(b) = (b-1) * (BulletLength/BulletAccuracy) + (BulletLength/BulletAccuracy) / 2;
        end

        break
    end
    
    % Начало моделирования
    for d = 1:CounterCoil
        for a=StartTime:length(ToqueDischargeTimeArray)-1
            TimeStep = ToqueDischargeTimeArray(a+1) - ToqueDischargeTimeArray(a);
            TimeStepArray(a) = TimeStep;
            for b = 1:BulletAccuracy
                for c = 1:CoilAccuracy
                     distance = InductorLocation(c,d)-(BulletLocation(b)+StartCoordinates);
                     H_tension = (Toque * InductorRadius(d) * distance) / (2 * ((InductorRadius(d)-BulletRadius)^2 + (distance^2))^1.5)*QuantityCoil(d)/CoilAccuracy;
                     H_from_Coil(c) = H_tension;
                end
                H_from_Bullet(b) = sum(H_from_Coil)/BulletAccuracy;
            end
            B_from_Bullet = sum(H_from_Bullet)*mu * mu0;
            if B_from_Bullet > MaxB
                B_from_Bullet = MaxB;
            end
            ABS = sum(B_from_Bullet)/abs(sum(B_from_Bullet));
            if sum(B_from_Bullet) == 0
                ABS = 1;
            end
            F = sum(B_from_Bullet)^2*2*pi*BulletRadius*BulletLength/(2*mu*mu0)*ABS;
            SupportArray(a) = sum(H_from_Bullet);
            Acceleration = F/Weight;
            AccelerationArray(a) = Acceleration;
            dV = Acceleration*TimeStep;
            Speed = StartSpeed+dV;
            StartSpeed = Speed;
            SpeedArray(a) = Speed;
            dx = Speed*TimeStep;
            Coordinate = StartCoordinates+dx;
            StartCoordinates = Coordinate;
            CoordinateArray(a) = Coordinate;
            (InductorStartPosition(d)+InductorLength(d)*0.4)-(Coordinate+BulletLength/2)
            if (InductorStartPosition(d)+InductorLength(d)*0.4)-(Coordinate+BulletLength/2)<0
                a
                StartTime = a+1;
                break
            end
        end 
        
        % Равномерное движение
        for a = StartTime:length(ToqueDischargeTimeArray)-1
    
            TimeStep = ToqueDischargeTimeArray(a+1) - ToqueDischargeTimeArray(a);
            TimeStepArray(a) = TimeStep;
            Acceleration = 0;
            AccelerationArray(a) = Acceleration;
            dV = Acceleration*TimeStep;
            Speed = StartSpeed+dV;
            StartSpeed = Speed;
            SpeedArray(a) = Speed;
            dx = Speed*TimeStep;
            Coordinate = StartCoordinates+dx;
            StartCoordinates = Coordinate;
            CoordinateArray(a) = Coordinate;
            (InductorStartPosition(d+1)-InductorLength(d+1)*0.4)-(Coordinate+BulletLength/2)
            if (InductorStartPosition(d+1)-InductorLength(d+1)*0.4)-(Coordinate+BulletLength/2)<0
                a
                StartTime = a+1;
                break
            end
        end
    end
    
    %графики
    while 1
    % 
    AccelerationArray(length(ToqueDischargeTimeArray))=0;
    SpeedArray(length(ToqueDischargeTimeArray))=0;
    CoordinateArray(length(ToqueDischargeTimeArray))=0;
    % График ускорения    
    subplot(3,1,1)
    plot(ToqueDischargeTimeArray, AccelerationArray);
    title ('ускорение')
    xlabel('t,c');
    ylabel('a,м/с^2');

    % График скорости
    subplot(3,1,2)
    plot(ToqueDischargeTimeArray, SpeedArray);
    title ('скорость')
    xlabel('t,c');
    ylabel('V,м/с');

    % График координат
    subplot(3,1,3)
    plot(ToqueDischargeTimeArray, CoordinateArray);
    title ('координаты')
    xlabel('t,c');
    ylabel('x,м');
    break
    end