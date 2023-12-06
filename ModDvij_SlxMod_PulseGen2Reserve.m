    clc;clear;
    % Начальные данные
    while 1
        % Параметры симулинка
        LaunchTime = 0;
        StopTime = 0.1;
        ChargeTime = 0.02;

        % резистивные составляющие
        Resistance = 74;

        % Параметры источника питания
        Voltage = 50;

        % Параметры конденсатора
        Capasity = 2000e-6;

        % Массивы параметров катушек
        CounterCoil = 2;
        InductorLength = zeros(CounterCoil+1,1);
        InductorRadius = zeros(CounterCoil+1,1);
        QuantityCoil = zeros(CounterCoil+1,1);
        InductorStartPosition = zeros(CounterCoil+1,1);
        Inductance = zeros(CounterCoil+1,1);
        % Параметры катушек
        for a = 1:CounterCoil+1
            InductorLength(a) = 58/1000;
            InductorRadius(a) = 8.5/1000;
            QuantityCoil(a) = 4000;
            InductorStartPosition(a) = ((a-1)*(InductorLength(a)*1000+10))/1000;
            Inductance(a) = 52255e-6;
        end
        % Мнимая послепоследняя катушка
        InductorStartPosition(CounterCoil+1) = 1000;

        % парметры снаряда
        BulletRadius = 5/1000;
        mu = 2500;
        BulletLength = InductorLength(1)*3/4;
        StartCoordinates = InductorStartPosition(1)-InductorLength(1)*0.4;
        Plotnost = 7800;
        Weight = pi*BulletLength*BulletRadius^2*Plotnost;
        MaxB = 2.12;
        BulletSquareCut = pi*BulletRadius^2;

        % Остальные
        mu0 = 4*pi*10^-7;
        CoilAccuracy = fix(QuantityCoil(a));
        BulletAccuracy = 50;
        StartSpeed=0;
        StartTime = 1;
        break
    end
    
    % Начальные параметры ключей для успешного запуска кода
    while 1
            SupportTimeArray = linspace(LaunchTime, StopTime, 578)';
            SwitchTime = zeros(length(SupportTimeArray),1+1+CounterCoil);
            SwitchTime(:,1) = SupportTimeArray;
            SwitchTrigger = zeros(1,2+CounterCoil*2);
            SwitchTrigger(1) = LaunchTime;
            SwitchTrigger(2) = ChargeTime;
            SwitchTrigger(3) = ChargeTime;

            for a = 4:2+CounterCoil*2
                SwitchTrigger(a) = StopTime;
            end

    
            % Формирование сигнала для зарядки
            for a = 1:1+CounterCoil
                for b = 1:length(SupportTimeArray)
                    if ((SupportTimeArray(b)>SwitchTrigger(a*2-1))&&(SupportTimeArray(b)<SwitchTrigger(a*2)))
                    SwitchTime(b,a+1) = 1;
                    else 
                        SwitchTime(b,a+1) = 0;
                    end
                    
                end
            end
            break
    end
    
    % Запуск Simulink модели и запись токов
    while 1

        sim('DiplomSimAnalog2Reserve.slx');
        ToqueDischargeTimeArray = ans.tout;
        ToqueDischargeArray = zeros(length(ToqueDischargeTimeArray),CounterCoil);
        for a=1:CounterCoil
            for b=1:length(ToqueDischargeTimeArray)
                ToqueDischargeArray(b,a) = ans.I.signals.values(b,a);
            end
        end
        break
    end
    
    % Параметры ключей
    while 1
        
            SwitchTime = zeros(length(ToqueDischargeTimeArray),1+CounterCoil+1);
            SwitchTime(:,1) = ToqueDischargeTimeArray;
            SwitchTrigger = zeros(1,2+CounterCoil*2);
            SwitchTrigger(1) = LaunchTime;
            SwitchTrigger(2) = ChargeTime;
            SwitchTrigger(3) = ChargeTime;

            for a = 4:2+CounterCoil*2
                SwitchTrigger(a) = StopTime;
            end

    
            % Формирование сигналов
            for a = 1:1+CounterCoil
                for b = 1:length(ToqueDischargeTimeArray)
                    if ((ToqueDischargeTimeArray(b)>SwitchTrigger(a*2-1))&(ToqueDischargeTimeArray(b)<SwitchTrigger(a*2)))
                    SwitchTime(b,a+1) = 1;
                    else 
                        SwitchTime(b,a+1) = 0;
                    end
                    
                end
            end
            break
    end
    
    % Создание массивов
    while 1
        
        InductorLocation = zeros(CoilAccuracy,CounterCoil);
        BulletLocation = zeros(BulletAccuracy,1);
        H_from_Coil = zeros(CoilAccuracy,1);
        H_from_Bullet = zeros(BulletAccuracy,1);
        AccelerationArray = zeros(length(ToqueDischargeTimeArray),1);
        SpeedArray = zeros(length(ToqueDischargeTimeArray)+1,1);
        CoordinateArray = zeros(length(ToqueDischargeTimeArray)+1,1);
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
                     H_tension = (ToqueDischargeArray(a,d) * InductorRadius(d) * distance) / (2 * ((InductorRadius(d))^2 + (distance^2))^1.5)*QuantityCoil(d)/CoilAccuracy;
                     H_from_Coil(c) = H_tension;
                end
                H_from_Bullet(b) = sum(H_from_Coil)/BulletAccuracy;
            end
            B_from_Bullet = sum(H_from_Bullet)*mu * mu0;
            ABS = sum(B_from_Bullet)/abs(sum(B_from_Bullet));
            if sum(B_from_Bullet) == 0
                ABS = 1;
            end
            F = sum(B_from_Bullet)^2*BulletSquareCut/(2*mu*mu0)*ABS;
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
            if (InductorStartPosition(d)+InductorLength(d)*0.4)-(Coordinate+BulletLength/2)<0
                SwitchTrigger(2+d*2) = ToqueDischargeTimeArray(a);
    
                % Формирование сигналов
                    SwitchTime = zeros(length(ToqueDischargeTimeArray),1+CounterCoil+1);
                    SwitchTime(:,1) = ToqueDischargeTimeArray;
                    for g = 1:CounterCoil+1
                        for f = 1:length(ToqueDischargeTimeArray)
                            if ((ToqueDischargeTimeArray(f)>SwitchTrigger(g*2-1))&&(ToqueDischargeTimeArray(f)<SwitchTrigger(g*2)))
                            SwitchTime(f,g+1) = 1;
                            else 
                                SwitchTime(f,g+1) = 0;
                            end
                            
                        end
                    end

                sim('DiplomSimAnalog2Reserve.slx');
                ToqueDischargeTimeArray = ans.tout;
                ToqueDischargeArray = zeros(length(ToqueDischargeTimeArray),CounterCoil);
                for g=1:CounterCoil
                    for b=1:length(ToqueDischargeTimeArray)
                        ToqueDischargeArray(b,g) = ans.I.signals.values(b,g);
                    end
                end
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
            if (InductorStartPosition(d+1)-InductorLength(d+1)*0.4)-(Coordinate+BulletLength/2)<0
                SwitchTrigger(2+d*2+1) = ToqueDischargeTimeArray(a);
                % Формирование сигнала для замыкания со второй катушкой

                % Формирование сигналов
                    SwitchTime = zeros(length(ToqueDischargeTimeArray),1+CounterCoil+1);
                    SwitchTime(:,1) = ToqueDischargeTimeArray;
                    for g = 1:CounterCoil+1
                        for f = 1:length(ToqueDischargeTimeArray)
                            if ((ToqueDischargeTimeArray(f)>SwitchTrigger(g*2-1))&&(ToqueDischargeTimeArray(f)<SwitchTrigger(g*2)))
                            SwitchTime(f,g+1) = 1;
                            else 
                                SwitchTime(f,g+1) = 0;
                            end
                            
                        end
                    end

                sim('DiplomSimAnalog2Reserve.slx');
                ToqueDischargeTimeArray = ans.tout;
                ToqueDischargeArray = zeros(length(ToqueDischargeTimeArray),CounterCoil);
                for g=1:CounterCoil
                    for b=1:length(ToqueDischargeTimeArray)
                        ToqueDischargeArray(b,g) = ans.I.signals.values(b,g);
                    end
                end
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
    W1 = Capasity*Voltage^2/2;
    W2 = Weight*Speed^2/2;
    KPD = W2/W1