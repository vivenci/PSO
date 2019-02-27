using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace aco.tools.Algorithm.PSO
{
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
    }

}
