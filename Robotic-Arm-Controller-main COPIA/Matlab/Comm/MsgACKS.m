classdef MsgACKS < Message
    properties (Access = private)

    end

    
    % Constructor
    methods (Access = public)
        function obj = MsgACKS(num)
            arguments
                num (1,1) {mustBeInteger} = 0;
            end
            
            obj@Message(Code.ACKS, num);
        end
    end

    
    % Getters
    methods (Access = public)
        function count = getCount(obj)
            arguments
                obj (1,1) MsgACKS;
            end

            count = obj.getNum() + 1;
        end

        function index = getIndex(obj)
            arguments
                obj (1,1) MsgACKS;
            end

            index = obj.getNum();
        end
    end

    
    % Setters
    methods (Access = public)
        function res = setCount(obj, count)
            arguments 
                obj (1,1) MsgACKS;
                count (1,1) {mustBeInteger};
            end
            
            res = obj.setNum(count - 1);
        end

        function res = setIndex(obj, index)
            arguments 
                obj (1,1) MsgACKS;
                index (1,1) {mustBeInteger};
            end
            
            res = obj.setNum(index);
        end
    end
    

    % Data Buffer payload
    methods (Access = protected)
        function dim = bsize_payload(obj) %#ok<MANU>
            arguments
                obj (1,1) MsgACKS;
            end

            dim = 0;
        end
        
        function res = parse_payload(obj, data)
            arguments
                obj (1,1) MsgACKS;
                data (1,:) uint8;
            end
            
            if(length(data) ~= obj.bsize_payload())
                res = false;
                return;
            end

            res = true;
        end

        function data = bytes_payload(obj)
            arguments
                obj (1,1) MsgACKS;
            end

            data = zeros([1, obj.bsize_payload()], 'uint8');
        end
    end
end