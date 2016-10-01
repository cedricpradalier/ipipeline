#ifndef IPL_CONDITIONS_H
#define IPL_CONDITIONS_H


/**
 * \class Condition
 * A condition is any class with a isVerified function
 * **/
class Condition {
	public :
		virtual bool isVerified() const = 0;
		virtual ~Condition() {}
};

/**
 * \class BoolCondition
 * A condition linked to a bool variable
 * **/
class BoolCondition : public Condition 
{
	protected :
		const bool & var;
	public :
		BoolCondition(const bool & v) : var(v) {}
		virtual bool isVerified() const {return var;}
};

/**
 * \class BoolNegCondition
 * A condition linked to the negation of a bool variable
 * **/
class BoolNegCondition : public Condition 
{
	protected :
		const bool & var;
	public :
		BoolNegCondition(const bool & v) : var(v) {}
		virtual bool isVerified() const {return !var;}
};

/**
 * \class EqCondition
 * A condition linked to a variable of type C and a value of type C,
 * verified when the variable equals the value, with operator ==
 * **/
template <class C>
class EqCondition : public Condition 
{
	protected :
		const C & var;
		C value;
	public :
		EqCondition(const C & v, const C & val) : var(v), value(val) {}
		virtual bool isVerified() const {
			//printf("EqCondition %d =? %d\n",var,value);
			return var==value;
		}
};

/**
 * \class NeqCondition
 * A condition linked to a variable of type C and a value of type C,
 * verified when the variable differs from the value, with operator !=
 * **/
template <class C>
class NeqCondition : public Condition 
{
	protected :
		const C & var;
		C value;
	public :
		NeqCondition(const C & v, const C & val) : var(v), value(val) {}
		virtual bool isVerified() const {return var!=value;}
};

/**
 * \class LtCondition
 * A condition linked to a variable of type C and a value of type C,
 * verified when the variable is strictly lower than the value, 
 * with operator <
 * **/
template <class C>
class LtCondition : public Condition 
{
	protected :
		const C & var;
		C value;
	public :
		LtCondition(const C & v, const C & val) : var(v), value(val) {}
		virtual bool isVerified() const {return var<value;}
};

/**
 * \class LeCondition
 * A condition linked to a variable of type C and a value of type C,
 * verified when the variable is lower than or equal to the value, 
 * with operator <=
 * **/
template <class C>
class LeCondition : public Condition 
{
	protected :
		const C & var;
		C value;
	public :
		LeCondition(const C & v, const C & val) : var(v), value(val) {}
		virtual bool isVerified() const {return var<=value;}
};

/**
 * \class GtCondition
 * A condition linked to a variable of type C and a value of type C,
 * verified when the variable is strictly greater than the value, 
 * with operator >
 * **/
template <class C>
class GtCondition : public Condition 
{
	protected :
		const C & var;
		C value;
	public :
		GtCondition(const C & v, const C & val) : var(v), value(val) {}
		virtual bool isVerified() const {return var>value;}
};

/**
 * \class LeCondition
 * A condition linked to a variable of type C and a value of type C,
 * verified when the variable is greater than or equal to the value, 
 * with operator >=
 * **/
template <class C>
class GeCondition : public Condition 
{
	protected :
		const C & var;
		C value;
	public :
		GeCondition(const C & v, const C & val) : var(v), value(val) {}
		virtual bool isVerified() const {return var>=value;}
};

/**
 * \class AndCondition
 * A nice way to AND conditions 
 * **/
class AndCondition : public Condition 
{
	protected : 
		Condition *cnd1, *cnd2;
	public :
		AndCondition(Condition * c1, Condition * c2) :
			cnd1(c1), cnd2(c2) {}
		virtual ~AndCondition() {delete cnd1;delete cnd2;}
		virtual bool isVerified() const {
			return cnd1->isVerified() && cnd2->isVerified();
		}
};

/**
 * \class OrCondition
 * A nice way to OR conditions 
 * **/
class OrCondition : public Condition 
{
	protected : 
		Condition *cnd1, *cnd2;
	public :
		OrCondition(Condition * c1, Condition * c2) :
			cnd1(c1), cnd2(c2) {}
		virtual ~OrCondition() {delete cnd1;delete cnd2;}
		virtual bool isVerified() const {
			return cnd1->isVerified() || cnd2->isVerified();
		}
};


#endif // IPL_CONDITIONS_H
