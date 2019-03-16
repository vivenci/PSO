using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace aco.tools.Algorithm.PSO
{
    [Serializable]
    public class PSOSolution
    {
        private PSOModel model;
        public PSOSolution(PSOModel m)
        {
            this.model = m;
        }

        public PSOResult GetResult()
        {
            if (model == null || model.Global == null || model.Goal == null)
            {
                return null;
            }
            else
            {
                PSOResult r = new PSOResult();
                r.BestSolu = model.Global.GlobalBestPosition;
                r.BestGoalValue = model.Goal.Calc(model.Global.GlobalBestPosition);
                return r;
            }
        }
    }

    public class PSOResult
    {
        public double[] BestSolu
        {
            get;
            set;
        }

        public double BestGoalValue
        {
            get;
            set;
        }

        public override string ToString()
        {
            string s = string.Format("在[{0}]处取得结果: {1}", string.Join(",", BestSolu.Select(p => p.ToString())), BestGoalValue);
            return s;
        }
    }

}
