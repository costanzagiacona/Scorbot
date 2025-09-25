classdef Robot < handle
    properties (Access = private)
        Comm Communication;         % Serial communication
        N (1,1) uint8;              % Robot size
        Endstop (:,1) logical;      % Robot endstops value
        EncsRob (:,1) int32;        % Robot encoders value
        EncsRef (:,1) int32;        % Setpoint encoders value
        Timeout (1,1) uint32;       % Communication timeout
    end
    
    
    % Constructor
    methods
        function obj = Robot(n, ts_us, varargin)
            arguments
                n (1,1) {mustBeInteger, mustBePositive};
                ts_us (1,1) {mustBeInteger, mustBeNonnegative};
            end
            arguments (Repeating)
                varargin;
            end

            if(n <= 0)
                error("Number of motors must be at least 1.");
            end
            
            if(n > 8)
                error("The protocol support up to 8 motors.");
            end
            
            obj.Comm = Communication(varargin{:});
            obj.Comm.flush();

            obj.N = uint8(n);
            obj.Endstop = false([obj.N, 1]);
            obj.EncsRob = zeros([obj.N, 1], 'int32');
            obj.EncsRef = zeros([obj.N, 1], 'int32');
            obj.Timeout = uint32(ts_us);
        end
    end
    
    
    % Getters
    methods (Access = public)
        function n = getSize(obj)
            arguments
                obj (1,1) Robot;
            end

            n = obj.N;
        end
        
        function endstops = getEndstops(obj)
            arguments
                obj (1,1) Robot;
            end

            endstops = obj.Endstop(1:obj.N, 1);
        end

        function encoders = getEncoders(obj)
            arguments
                obj (1,1) Robot;
            end

            encoders = obj.EncsRob(1:obj.N, 1);
        end
    end
    
    
    % Communication
    methods (Access = public)
        function res = ctrl_idle(obj)
            arguments
                obj (1,:) Robot;
            end

            msg = MsgIDLE();
            msg.setCount(obj.N);

            obj.Comm.snd(msg);
            [res, msg] = obj.Comm.rcv(2*obj.Timeout);

            if(res && msg.getCode() == Code.ACKC && msg.getCount() == obj.N)
                for k = 1:obj.N
                    obj.EncsRob(k) = obj.EncsRob(k) + int32(msg.getDeltaEnc(k));
                    obj.Endstop(k) = msg.getEndStop(k);
                end
            else
                res = false;
                obj.Comm.flush();
            end
        end

        function res = ctrl_pwm(obj, pwms)
            arguments
                obj (1,1) Robot;
                pwms (1,:) {mustBeNumeric};
            end

            if(length(pwms) ~= obj.N)
                error("Wrong pwms length.");
            end

            msg = MsgPWM();
            msg.setCount(obj.N);
            for k = 1:obj.N
                msg.setPwm(k, int16(pwms(k)));
            end

            obj.Comm.snd(msg);
            [res, msg] = obj.Comm.rcv(2*obj.Timeout);

            if(res && msg.getCode() == Code.ACKC && msg.getCount() == obj.N)
                for k = 1:obj.N
                    obj.EncsRob(k) = obj.EncsRob(k) + int32(msg.getDeltaEnc(k));
                    obj.Endstop(k) = msg.getEndStop(k);
                end
            else
                res = false;
                obj.Comm.flush();
            end
        end

        function res = ctrl_ref(obj, encs)
            arguments
                obj (1,1) Robot;
                encs (1,:) {mustBeNumeric};
            end

            if(length(encs) ~= obj.N)
                error("Wrong encs length.");
            end
            
            delta_ref = zeros([obj.N, 1], 'int16');

            msg = MsgREF();
            msg.setCount(obj.N);
            for k = 1:obj.N
                msg.setDeltaEnc(k, int32(encs(k)) - obj.EncsRef(k));
                delta_ref(k) = msg.getDeltaEnc(k);
            end

            obj.Comm.snd(msg);
            [res, msg] = obj.Comm.rcv(2*obj.Timeout);
            
            if(res && msg.getCode() == Code.ACKC && msg.getCount() == obj.N)
                for k = 1:obj.N
                    obj.Endstop(k) = msg.getEndStop(k);
                    obj.EncsRob(k) = obj.EncsRob(k) + int32(msg.getDeltaEnc(k));
                    obj.EncsRef(k) = obj.EncsRef(k) + int32(delta_ref(k));
                end
            else
                res = false;
                obj.Comm.flush();
            end
        end

        function res = setup_robot(obj, ts_us, ticks)
            arguments
                obj (1,1) Robot;
                ts_us (1,1) {mustBeNumeric, mustBeNonnegative};
                ticks (1,1) {mustBeNumeric, mustBeNonnegative};
            end

            msg = MsgROBOT();
            msg.setCount(obj.N);
            msg.setTimeSampling(ts_us);
            msg.setAllowedTicks(ticks);
            ts_us = msg.getTimeSampling();

            obj.Comm.snd(msg);
            [res, msg] = obj.Comm.rcv(2*obj.Timeout);
            
            if(res && msg.getCode() == Code.ACKS && msg.getCount() == obj.N)
                obj.Timeout = uint32(ts_us);
            else
                res = false;
                obj.Comm.flush();
            end
        end

        function res = setup_motor(obj, index, spin_dir, enc_dir, enc)
            arguments
                obj (1,1) Robot;
                index (1,1) {mustBeInteger, mustBeNonnegative};
                spin_dir (1,1) {mustBeInteger};
                enc_dir (1,1) {mustBeInteger};
                enc (1,1) {mustBeInteger};
            end

            if(index < 1 || index > obj.N)
                error("Index out of bound.")
            end

            msg = MsgMOTOR();
            msg.setIndex(index-1);
            msg.setChangeEncoder(true);
            msg.setInvertSpinDir(spin_dir < 0);
            msg.setChangeSpinDir(spin_dir ~= 0);
            msg.setInvertEncDir(enc_dir < 0);
            msg.setChangeEncDir(enc_dir ~= 0);
            msg.setEncoderValue(enc);
            enc = msg.getEncoderValue();

            obj.Comm.snd(msg);
            [res, msg] = obj.Comm.rcv(2*obj.Timeout);
            
            if(res && msg.getCode() == Code.ACKS && msg.getIndex() == (index-1))
                obj.EncsRob(index) = int32(enc);
                obj.EncsRef(index) = int32(enc);
            else
                res = false;
                obj.Comm.flush();
            end
        end

        function res = setup_pid(obj, index, div, kp, ki, kd, sat, pole)
            arguments
                obj (1,1) Robot;
                index (1,1) {mustBeInteger, mustBeNonnegative};
                div (1,1) {mustBeNumeric};
                kp (1,1) {mustBeNumeric};
                ki (1,1) {mustBeNumeric};
                kd (1,1) {mustBeNumeric};
                sat (1,1) {mustBeNumeric};
                pole (1,1) {mustBeNumeric};
            end

            if(index < 1 || index > obj.N)
                error("Index out of bound.")
            end

            msg = MsgPID();
            msg.setIndex(index-1);
            msg.setPidDiv(div);
            msg.setPidKp(kp);
            msg.setPidKi(ki);
            msg.setPidKd(kd);
            msg.setPidSat(sat);
            msg.setPidPole(pole);

            obj.Comm.snd(msg);
            [res, msg] = obj.Comm.rcv(2*obj.Timeout);
            
            if(res && msg.getCode() == Code.ACKS && msg.getIndex() == (index-1))
            else
                res = false;
                obj.Comm.flush();
            end
        end
    end


    % Reset
    methods (Access = public)
        function res = reset(obj)
            arguments
                obj (1,1) Robot;
            end

            obj.Comm.flush();
            
            res = obj.setup_robot(obj.Timeout);
            
            for k = 1:obj.N
                if(~res)
                    break;
                end
                res = obj.setup_motor(k, 0, 0, 0);
                if(res)
                    obj.EncsRob(k) = int32(0);
                    obj.EncsRef(k) = int32(0);
                end
            end

            for k = 1:obj.N
                if(~res)
                    break;
                end
                res = obj.setup_pid(k, 1, 0, 0, 0, 0, 0);
            end

            if(~res)
                obj.Comm.flush();
            end
        end
    end
end
