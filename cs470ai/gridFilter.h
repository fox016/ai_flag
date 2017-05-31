
class GridFilter
{
private:
	double truePositive;
	double trueNegative;
public:
	GridFilter(double pos, double neg)
	{
		truePositive = pos;
		trueNegative = neg;
	}
};
