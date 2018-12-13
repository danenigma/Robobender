classdef rangeImage < handle

    properties(Constant)
    ANGLE_OFFSET = 3*pi/180;
    PALLET_WIDTH = 0.15;
    
    end
    
    properties(Access = public)
    
    end
    methods(Access = public)
        
        function obj = rangeImage()
        end
        function [is_pallet, pallet_pos, lambda]  = isPallet(obj, x, y)
            is_pallet = false;
            pallet_pos = [0;0;0];
            numPts = length(x);
            mu_x = mean(x);
            mu_y = mean(y);
            x = x - mu_x;
            y = y - mu_y;
            Ixx = x' * x;
            Iyy = y' * y;
            Ixy = - x' * y;
            
            
            Inertia = [Ixx Ixy; Ixy Iyy] / numPts; % normalized
            lambda = eig(Inertia);
            lambda = sqrt(lambda)*1000.0;
            if lambda(1) < 1.3 
                th = atan2(2*Ixy, Iyy-Ixx)/2;
                pallet_pos = [mu_x;mu_y;th];
                is_pallet = true;
            end
        end
        function pallet_pos = findLineCandidate(obj, range_im, min_num_pts)
            pallet_pos = [0;0;0];
            pixels = length(range_im);
            goodOnes = range_im > 0.06 & range_im < 1;
            range_im = range_im(goodOnes);
            indices = linspace(1,pixels,pixels)';
            indices = indices(goodOnes);
            % Compute the angles of surviving points
            ths = indices;
            
            for pixel = 1:length(range_im)
                %get the points to the left and right of the center point
                %pixel
                l_r_points = obj.getLeftAndRightPixels(pixel, range_im, ths);
               
                [x, y, ~] = irToXy(ths(l_r_points)', range_im(l_r_points));
                %number of points 
                numPts = length(x)-2;

                if numPts >= min_num_pts
                    [c_is_pallet, c_pallet_pos, c_lambda]  = obj.isPallet(x(2:end-1), y(2:end-1));
                    
                    if ~c_is_pallet
                        continue;
                    end
                    
                    [l_is_pallet, l_pallet_pos, l_lambda]  = obj.isPallet(x(1:end-1), y(1:end-1));
                    if l_is_pallet
                        continue;
                    end
                    [r_is_pallet, r_pallet_pos, r_lambda]  = obj.isPallet(x(2:end), y(2:end));
                    if r_is_pallet
                        continue;
                    end
                    
                    pallet_pos = c_pallet_pos;
                    
                end
                
                %break
            end
        end
        function pallet_pos = findLineCandidateROI(obj, range_im, min_num_pts, angle)
            pallet_pos = [0;0;0];
            pixels = length(range_im);
            goodOnes = range_im > 0.06 & range_im < 1;
            indices  = linspace(1,pixels,pixels)';
            valid_angles = indices<angle | indices >360-angle;
            goodOnes = goodOnes & valid_angles;
            range_im = range_im(goodOnes);
            indices = indices(goodOnes);
            
            % Compute the angles of surviving points
            ths = indices;
            
            for pixel = 1:length(range_im)
                %get the points to the left and right of the center point
                %pixel
                l_r_points = obj.getLeftAndRightPixels(pixel, range_im, ths);
               
                [x, y, ~] = irToXy(ths(l_r_points)', range_im(l_r_points));
                %number of points 
                numPts = length(x)-2;

                if numPts >= min_num_pts
                    [c_is_pallet, pallet_pos, c_lambda]  = obj.isPallet(x(2:end-1), y(2:end-1));
                     
                end
                
                %break
            end
        end
  
        function pallet_pos = findLineCandidateInROI(obj, range_im, min_num_pts, angle)
            pallet_pos = [0;0;0];
            
            range_im = [range_im(1:angle); range_im(360-angle:end)];
            indices = [(1:angle) (360-angle:360)];
            goodOnes = range_im > 0.06 & range_im < 0.8;%was 1.
            range_im = range_im(goodOnes);
            indices = indices(goodOnes);
            ths = indices';
            
            for pixel = 1:length(range_im)
                %get the points to the left and right of the center point
                %pixel
                l_r_points = obj.getLeftAndRightPixels(pixel, range_im, ths);
               
                [x, y, ~] = irToXy(ths(l_r_points)', range_im(l_r_points));
                %number of points
                numPts = length(x)-2;

                if numPts >= min_num_pts
                    [is_pallet, pallet_pos, lambda]  = obj.isPallet(x, y);
                    break    
                    
                end
                
            end
        end
               
        function [ x, y, b] = irToXy(obj, i, r )
        % irToXy finds position and bearing of a range pixel endpoint
        % Finds the position and bearing of the endpoint of a range pixel in the plane.
        % Fill in code here

        th = (i-1)*pi/180-rangeImage.ANGLE_OFFSET ;
        th = reshape(th, [length(i), 1]);
        x  = r.*cos(th);
        y  = r.*sin(th);
        b  = atan2(y,x);
        end
        function points = getLeftAndRightPixels(obj, idx, range_im, ths)
            %go right  till length/2
            points = [idx];
            N = length(range_im);
            r  = range_im(idx);
            th = ths(idx);
            L = idx;
            R = idx;
            [x, y, ~] = obj.irToXy(th, r);
            r_vec = [x; y];
            %get points to the left of the central point
            for i=1:N-1
                
                index  = mod(idx-1-i, N);
                
                if index < 0 
                    index = index + N;
                end
                r_i = range_im(index+1);
                th_i = ths(index+1);
                
                [x_i, y_i, ~] = obj.irToXy(th_i, r_i);
                r_i_vec = [x_i; y_i];
                
                %distance from the center point.
                distance = norm(r_i_vec-r_vec);
                
                if distance > rangeImage.PALLET_WIDTH/2                    
                    L = index + 1;
                    break
                else
                    points = [points, index+1];
                    
                end
            end
            %get points to the right of the central point
            
            for i=1:N-1
                
                index  = mod(idx-1+i, N);
                r_i = range_im(index+1);
                
                th_i = ths(index+1);
                
                [x_i, y_i, ~] = obj.irToXy(th_i, r_i);
                r_i_vec = [x_i; y_i];
                distance = norm(r_i_vec-r_vec);
                
                if distance > rangeImage.PALLET_WIDTH/2
                    R = index + 1;
                    break
                else
                    points = [points, index+1];
                end
            end

        points = unique(points);
        points = [L, points, R];
        end
    end

end