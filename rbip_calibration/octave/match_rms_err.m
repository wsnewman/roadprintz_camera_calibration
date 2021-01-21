## Copyright (C) 2021 Wyatt
## 
## This program is free software; you can redistribute it and/or modify it
## under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 3 of the License, or
## (at your option) any later version.
## 
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
## 
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.

## -*- texinfo -*- 
## @deftypefn {} {@var{retval} =} match_rms_err (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: Wyatt <wsn@ava>
## Created: 2021-01-16

%rms_err for matching sets of 2D points
function [rms_err] = match_rms_err (pts1, pts2)
  pts_diff = pts1-pts2;
  mat_size = size(pts_diff);
  npts = mat_size(1);
  %want these in order (x,y);(x2,y2);...
  if (mat_size(2)>mat_size(1))
    pts_diff=pts_diff';
    npts = mat_size(2);
  end
  %npts
  sum_sqd_err = 0;
  for i=1:npts
    sum_sqd_err = sum_sqd_err+pts_diff(i,1)*pts_diff(i,1)+pts_diff(i,2)*pts_diff(i,2);
  end
  rms_err = sqrt(sum_sqd_err/npts);

endfunction
