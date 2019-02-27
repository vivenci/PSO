using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace aco.tools.Algorithm.PSO
{
    /// <summary>
    /// 约束条件验证类
    /// </summary>
    public class ConstraintVerification
    {
        public List<IVerification> constraints = null;

        public ConstraintVerification(List<IVerification> cons)
        {
            this.constraints = cons;
        }

        /// <summary>
        /// 验证IVerification接口
        /// </summary>
        /// <param name="paras">向接口对象传递的参数列表</param>
        /// <returns>是否验证成功</returns>
        public bool Verification(double[] paras)
        {
            bool flag = true;
            if (this.constraints != null)
            {
                for (int i = 0; i < constraints.Count; i++)
                {
                    var c = constraints[i];
                    object[] args = paras.Select(p => (object)p).ToArray();
                    var ret = c.GetResult(args);
                    bool? fi = null;
                    if (c.EnableAfterAction)
                    {
                        fi = c.AfterAction(ret);
                    }
                    else
                    {
                        fi = (bool)ret;
                    }
                    flag = flag && fi.Value;
                    if (!flag)
                    {
                        break;
                    }
                }
            }
            return flag;
        }
    }
}
