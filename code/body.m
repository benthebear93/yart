classdef body<handle
    % body class

    properties (Access = public)
        name
        Position % 3D position in space [x, y, z]
        Orientation % 3D orientation in space [roll, pitch, yaw]
        size
        T_val
        R
        mass
        inertia
        isStatic
        k
        b
        type
        force = [0; 0; 0];
        acc = [0; 0; 0];
        vel = [0; 0; 0];
        pos = [0; 0; 0];    
        acc_prev = [0; 0; 0];
        vel_prev = [0; 0; 0];
        pos_prev = [0; 0; 0];

        acc_com = [0; 0; 0];
        vel_com = [0; 0; 0];   
        pos_com = [0; 0; 0];

        acc_com_prev = [0; 0; 0];
        vel_com_prev = [0; 0; 0];   
        pos_com_prev = [0; 0; 0];

        ang_acc_com = [0; 0; 0];
        ang_vel_com = [0; 0; 0];
        ang_pos_com = [0; 0; 0];
        ang_acc_com_prev = [0; 0; 0];
        ang_vel_com_prev = [0; 0; 0];
        ang_pos_com_prev = [0; 0; 0];

        ang_acc = [0; 0; 0];
        ang_vel = [0; 0; 0]; 
        ang_pos = [0; 0; 0];   

        ang_acc_prev = [0; 0; 0];
        ang_vel_prev = [0; 0; 0];
        ang_pos_prev = [0; 0; 0];

    end
    methods
        function obj = body(name, mass, inertia, pos, ori, size, k, b, isStatic, type)
            % Constructor for the Finger class
            % pos - Initial position, e.g., [x, y, z]
            % ori - Initial orientation, e.g., [roll, pitch, yaw]
            obj.name = name;
            if nargin < 7
                k = 0.1;
            end
            if nargin < 8
                b = 0.1;
            end
            if nargin < 9
                isStatic = false;
            end
            if nargin < 10
                type = "cube"; % default type is cube
            end
            obj.mass = mass;
            obj.inertia = inertia;  % Inertia matrix
            obj.size = size;
            obj.T_val = pr2t(pos, rpy2r(ori));
            obj.isStatic = isStatic;
            obj.k = k;
            obj.b = b;
            obj.pos_com= pos;
            obj.ang_pos_com = ori;
            obj.pos = obj.pos_com - rpy2r(obj.ang_pos_com)*[0; 0; - obj.size(3)/2];
            obj.ang_pos = ori;
            obj.R = rpy2r(ori);
            obj.type = type;
            if obj.type=="solid"
                disp('Solid object is static by default');
                obj.isStatic = true;
            elseif obj.type=="cube"
                disp('Cube object is not static by default');
                obj.isStatic = false;
            elseif obj.type=="cuboid" 
                disp('cuboid object is not static by default');
                obj.isStatic = false;
            end
        end

        function obj = update(obj)
            if obj.isStatic
                obj.acc = [0; 0; 0];
                obj.vel = [0; 0; 0];
                obj.pos = [0; 0; 0];
                obj.acc_com = [0; 0; 0];
                obj.vel_com = [0; 0; 0];
                obj.pos_com = [0; 0; 0];
                obj.ang_acc_com = [0; 0; 0];
                obj.ang_vel_com = [0; 0; 0];
                obj.ang_pos_com = [0; 0; 0];
                obj.ang_acc = [0; 0; 0];
                obj.ang_vel = [0; 0; 0];
                obj.ang_pos = [0; 0; 0];
            else
                obj.T_val = pr2t(obj.pos_com, rpy2r(obj.ang_pos_com));
            end
        end
        function contact_vector = calc_contact_vector(obj, contact_point)
            contact_vector = obj.pos_com - contact_point;
        end
        function obj = move(obj, force, torque, dt, r2)
            % Basic single rigid body dynamics
            r = calc_contact_vector(obj, obj.pos);
            obj.vel_prev = obj.vel;
            obj.pos_prev = obj.pos;

            obj.acc = force/obj.mass;
            obj.vel = obj.vel_prev + obj.acc*dt;
            obj.pos = obj.pos_prev + obj.vel_prev*dt; 
            
            obj.ang_vel_prev = obj.ang_vel;
            obj.ang_pos_prev = obj.ang_pos;
        
            obj.ang_acc = obj.inertia\torque;
            obj.ang_vel = obj.ang_vel_prev  + obj.ang_acc*dt;
            obj.ang_pos = obj.ang_pos_prev + obj.ang_vel_prev *dt;

            obj.ang_vel_com_prev = obj.ang_vel_com;
            obj.ang_pos_com_prev = obj.ang_pos_com;
        
            obj.ang_acc_com = obj.ang_acc;
            obj.ang_vel_com = obj.ang_vel_com_prev + obj.ang_acc_com *dt;
            obj.ang_pos_com = obj.ang_pos_com_prev + obj.ang_vel_com_prev*dt;
        
            obj.vel_com_prev = obj.vel_com;
            obj.pos_com_prev = obj.pos_com;
            
            obj.acc_com = force/obj.mass;
            obj.vel_com = obj.vel_com_prev + obj.acc_com*dt + cross(obj.ang_vel_com_prev*dt, r) + cross(obj.ang_vel_com_prev*dt, r2);
            obj.pos_com = obj.pos_prev+ obj.vel_prev*dt + rpy2r(obj.ang_pos_com)*[0; 0; -obj.size(3)/2]; % FIXME : shouldn't ang pos com be previous one?

            %% Currently, we are only calculating pos_com with r, which is not correct. We also need to calculate vel_com based on r2.
        end

        function force = mkb_react(obj, penetrated_depth, vel_rel)
            % mkb_react - This function calculates the reaction force between the objects.
            spring_force = obj.k*penetrated_depth;
            damping_force = obj.b*vel_rel;
            fprintf("spring_force : %f %f %f \n", spring_force(1), spring_force(2), spring_force(3));
            fprintf("damping_force : %f %f %f \n", damping_force(1), damping_force(2), damping_force(3));
            force = -spring_force - damping_force;
        end


    end
end