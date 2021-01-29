## Copyright (C) 2020 Wyatt
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
## @deftypefn {} {@var{retval} =} dist (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: Wyatt <wsn@ava>
## Created: 2020-12-20

function [euc_dist] = dist (pt1, pt2)
  euc_dist = sqrt((pt2(1)-pt1(1))*(pt2(1)-pt1(1))+(pt2(2)-pt1(2))*(pt2(2)-pt1(2)));

endfunction
