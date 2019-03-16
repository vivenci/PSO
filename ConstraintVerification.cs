using aco.common.Logs;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace aco.tools.Algorithm.PSO
{
    /// <summary>
    /// 约束条件验证类
    /// </summary>
    [Serializable]
    public class ConstraintVerification
    {
        public List<IVerification> constraints = null;
        FileLogger logger = FileLogger.Create(LoggerSetting.DefaultSystemLogName);

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
                IVerification c = null;
                object[] args = new object[paras.Length];
                object ret = null;
                for (int i = 0; i < constraints.Count; i++)
                {
                    c = constraints[i];
                    args = paras.Select(p => (object)p).ToArray();
                    ret = c.GetResult(args);
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
                        string log = string.Format("指标[{0}]在[{1}]处无解.", c.Name, string.Join(",", paras.Select(p => (object)p)));
                        logger.WriteLine(log);
                        break;
                    }
                }
            }
            return flag;
        }
    }
}
