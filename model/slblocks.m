function blkStruct = slblocks
blkStruct.Name = ['KL_Uart'];
blkStruct.OpenFcn = 'KL_Uart';
blkStruct.MaskDisplay = '';

Browser(1).Library = 'KL_Uart';
Browser(1).Name    = 'KL_Uart';
Browser(1).IsFlat  = 0;% Is this library "flat" (i.e. no subsystems)?

blkStruct.Browser = Browser;

% End of slblocks

%
%  OPAL-RT Technologies inc
%
%  Copyright (C) 2000. All rights reserved
%  Internal m files

%  File name =         $Workfile: slblocks.m $
%  SourceSafe path =   $Logfile: /SIMUPAR/soft/Simulink/libR12/slblocks.m $
%  SourceSafe rev. =   $Revision: 2.6 $
%  Last checked in =   $Date: 2008/04/23 20:05:09 $
%  Last updated =      $Modtime: 10/10/03 2:38p $
%  Last modified by =  $Author: cristinao $

