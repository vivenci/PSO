using System;
using System.Collections.Generic;
using System.Linq;
using System.Linq.Expressions;
using System.Text;
using aco.tools.NFormula;

namespace aco.tools.Algorithm.PSO
{
    /// <summary>
    /// 粒子群目标函数
    /// (用于判定粒子是否找到目标)
    /// </summary>
    [Serializable]
    public class PSOGoal
    {
        /// <summary>
        /// 初始化目标
        /// </summary>
        /// <param name="exp">目标函数表达式</param>
        /// <param name="n">食物数目</param>
        public PSOGoal(string exp, int n = 3)
        {
            IEnumerable<ParameterExpression> paras = Formula.CreateParams(n);
            Formula f = new Formula(exp, paras);
            this.Expression = exp;
            this.GoalFunc = f.GetDelegate();
        }

        /// <summary>
        /// PSO模型函数的表达式
        /// </summary>
        public string Expression { get; set; }

        /// <summary>
        /// PSO模型函数
        /// </summary>
        public Delegate GoalFunc { get; set; }


        /// <summary>
        /// 计算函数值
        /// </summary>
        /// <param name="paras">当前自变量取值列表(当前各粒子的位置列表)</param>
        /// <returns>当前取值</returns>
        public double Calc(double[] paras)
        {
            double dr = double.NaN;
            if (this.GoalFunc != null)
            {
                object[] objs = paras.Select(p => (object)p).ToArray();
                object ret = this.GoalFunc.DynamicInvoke(objs);
                if (ret != null)
                {
                    double.TryParse(ret.ToString(), out dr);
                }
            }
            return dr;
        }


    }

}
