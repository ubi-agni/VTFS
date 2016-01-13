#include "maniptool.h"

ManipTool::ManipTool()
{
    mtt = Notool;
    ts.dof_num = 0;
    ts.init_ctc_x = -1;
    ts.init_ctc_y = -1;
    ts.cur_ctc_x = -1;
    ts.cur_ctc_y = -1;
    ts.rel_o.setZero();
}
