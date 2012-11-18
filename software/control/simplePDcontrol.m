classdef simplePDcontrol < AffineSystem
    methods
        function obj = simplePDcontrol(num_x,num_u,x_des)
            num_q = num_x/2;
            Ac = [];
            Bc = [];
            xcdot0 = [];
            Ad = [];
            Bd = [];
            xdn0 = [];
            C = [];
            if(num_u~=num_q)
                error('Not implemented yet')
            end
            D = [-10*eye(num_q) -10*eye(num_q)];
            y0 = -D*x_des;
            obj = obj@AffineSystem(Ac,Bc,xcdot0,Ad,Bd,xdn0,C,D,y0);
        end
    end
end