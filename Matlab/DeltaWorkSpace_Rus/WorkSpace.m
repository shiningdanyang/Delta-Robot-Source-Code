function [A, n] = WorkSpace()
n = 1; %êîëè÷åñòâî òî÷åê â ìàññèâå
% ïåðåáîð âñåõ âîçìîæíûõ ïîëîæåíèé îáúåêòà óïðàâëåíèÿ
h = 5;% øàã îáõîäà
A = zeros((140/h)^3, 3);
for th1 = -35 : h : 105
    for th2 = -35 : h : 105
        for th3 = -35 : h : 105
            [X, Y, Z, fl] = FKinem(th1, th2, th3);
            if fl == 0 
               A(n, 1) = X;
               A(n, 2) = Y;
               A(n, 3) = Z;
               n = n+1;
            end
         end
    end
end
end
