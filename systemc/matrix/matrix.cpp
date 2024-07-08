#include "matrix.h"

template <typename T>
Matrix<T>::Matrix(){
  resize( 0, 0 );

  _error = 0;
  _det_up_to_date = 0;
}

template <typename T>
Matrix<T>::Matrix( uint rows, uint columns ){
  resize( rows, columns );

  _error = 0;
  _det_up_to_date = 0;
}

template <typename T>
Matrix<T>::Matrix( vector<vector<T> > input ){
  resize( input.size(), input[0].size() ); //Eeeehhh....
  _data = input;
  _error = 0;
  _det_up_to_date = 0;
}

template <typename T>
Matrix<T>::Matrix( vector<T> input ){
  resize( 1, input.size() );
  _data[0] = input;
  _error = 0;
  _det_up_to_date = 0;
}

template <typename T>
Matrix<T>::Matrix( const Matrix& m, uint row ){
  resize( 1, m._data[0].size() );
  _data[0] = m._data[row];
  _error = 0;
  _det_up_to_date = 0;
}

template <typename T>
Matrix<T>::Matrix( const Matrix& m ){
  *this = m;
}

template <typename T>
Matrix<T>::~Matrix(){

}

template <typename T>
void Matrix<T>::resize( uint rows, uint cols ){
  _m_in = _n_in = _m_out = _n_out = 0; //Reset all IO iterators.
  _m = rows;
  _n = cols;
  if( _data.size() == 0 || _data[0].size() == 0 ){
    //The compiler should jump out after the first one, evading an error.
    _data.clear();
    _data.assign( rows, vector<T>(cols, 0.0 ) );
    return;
  }
  if( cols > _data[0].size() ){
    for( uint i = 0; i < cols-_data[0].size(); i++ ){
      for( uint j = 0; j < _data.size(); j++ ){
	_data[j].push_back( 0.0 );
      }
    }
  }
  else if ( cols < _data[0].size() ){
    for( uint i = 0; i < _data[0].size()-cols; i++ ){
      for( uint j = 0; j < _data.size(); j++ ){
	_data[j].pop_back();
      }
    }
  }
  if( rows > _data.size() ){
    for( uint i = 0; i < rows-_data.size(); i++ ){
      _data.push_back( vector<T>(cols, 0.0 ) );
    }
  }
  else if( rows < _data.size() ){
    for( uint i = 0; i < _data.size()-rows; i++ ){
      _data.pop_back();
    }
  }
}

template <typename T>
bool Matrix<T>::operator == ( const Matrix& rhs ){
  if( _m != rhs._m || _n != rhs._n || _error || rhs._error ){
    return 0;
  }
  bool result = 1;
  for( uint i = 0; i < m(); i++ ){
    for( uint j = 0; j < n(); j++ ){
      if( _data[i][j] != rhs._data[i][j] )
	result = false;
    }
  }
  return result;
}

template <typename T>
bool Matrix<T>::operator != ( const Matrix& rhs ){
  return !(*this == rhs);
}

template <typename T>
Matrix<T>& Matrix<T>::operator << ( T input ){
  //The IO iterators can be an issue here. Currently, resize() handles that.
  if( _m == 0 || _n == 0 || _error ){
    _print_error( "Attempting to input into an empty matrix." );
    _error = 1;
  }
  _data[_m_in][_n_in] = input;
  _n_in++;
  if( _n_in == _n ){
    _n_in = 0;
    _m_in++;
  }
  if( _m_in == _m ){
    _m_in = 0;
  }
  _det_up_to_date = 0;
  return *this;
}

template <typename T>
void Matrix<T>::clear(){
  _m_in = _n_in = _m_out = _n_out = 0; //Reset all IO iterators.
  _error = 0;
  for( uint i = 0; i < _m; i++ ){
    for( uint j = 0; j < _n; j++ ){
      _data[i][j] = 0.0;
    }
  }
  _det_up_to_date = 0;
}

template <typename T>
void Matrix<T>::print(){
  for( uint i = 0; i < _m; i++ ){
    for( uint j = 0; j < _n; j++ ){
      cout << _data[i][j] << " ";
    }
    cout << endl;
  }
}

template <typename T>
string Matrix<T>::print_test(){
  std::stringstream ss;
  for( uint i = 0; i < _m; i++ ){
    for( uint j = 0; j < _n; j++ ){
      ss << _data[i][j] << " ";
    }
  }
  string s = ss.str();
  return s.substr(0, s.size()-1);
}

template <typename T>
Matrix<T> Matrix<T>::operator - () const {
  Matrix result = *this;
  return result * -1;
}

template <typename T>
Matrix<T> Matrix<T>::addToRow( const Matrix& rhs, const uint row ){
  if( (row > 0 && row <= _m) || rhs._m != 1 ){
    Matrix result = *this;
    for( uint i = 0; i < _n; i++ ){
      result._data[row-1][i] = result._data[row-1][i] + rhs._data[0][i];
    }
    return result;
  }
  else {
    _error = 1;
    return *this;
  }
}

template <typename T>
Matrix<T>& Matrix<T>::ADDToRow( const Matrix& rhs, const uint row ){
  if( (row > 0 && row <= _m) || rhs._m != 1 ){
    for( uint i = 0; i < _n; i++ ){
      _data[row-1][i] = _data[row-1][i] + rhs._data[0][i];
    }
  }
  else {
    _error = 1;
  }
  return *this;
}
template <typename T>
Matrix<T> Matrix<T>::switchRows( const uint row1, const uint row2 ){
  Matrix result = *this;
  vector<T> temp = result._data[row1-1];
  result._data[row1-1] = result._data[row2-1];
  result._data[row2-1] = temp;
  return result;
}
template <typename T>
Matrix<T>& Matrix<T>::SWITCHRows( const uint row1, const uint row2 ){
  vector<T> temp = _data[row1-1];
  _data[row1-1] = _data[row2-1];
  _data[row2-1] = temp;
  return *this;
}

template <typename T>
Matrix<T> Matrix<T>::multiplyRow( const T d, const uint row ){
  if( row > 0 && row <= _m ){
    Matrix result = *this;
    for( uint i = 0; i < _n; i++ ){
      result._data[row-1][i] = result._data[row-1][i] * d;
    }
    return result;
  }
  else {
    _error = 1;
    return *this;
  }
}

template <typename T>
Matrix<T>& Matrix<T>::MULTIPLYRow( const T d, const uint row ){
  if( row > 0 && row <= _m ){
    for( uint i = 0; i < _n; i++ ){
      _data[row-1][i] = _data[row-1][i] * d;
    }
  }
  else {
    _error = 1;
  }
  return *this;
}

template <typename T>
Matrix<T> Matrix<T>::upperTriangular(){
  if( _m == _n ){
    Matrix result = *this;
    //*****For each leading 1...
    for( uint i = 0; i < _m; i++ ){
      //*****Put a leading nonzero in the current row.
      if( result._data[i][i] == 0 ){
	for( uint j = i+1; j < _m; j++ ){
	  if( result._data[j][i] != 0 ){
	    result.SWITCHRows( i+1, j+1 );
	    break;
	  }
	}
      }
      if( result._data[i][i] == 0 ){ //All 0s in the column. Not possible.
	_error = 1;
	return *this;
      }
      //*****Multiply current row by 1/value to give it a leading 1.
      T value = result._data[i][i];
      result.MULTIPLYRow( 1/value, i+1 );
      //*****Clean the column so the leading 1 is the only number in the column.
      Matrix res( result, i );
      for( uint j = 0; j < _m; j++ ){
	if( i != j ){
	  T coeff = result._data[j][i];
	  result.ADDToRow( (res*-coeff), j+1 );
	}
      }
    }
    return result;
  }
  else {
    _error = 1;
    return *this;
  }
}

template <typename T>
Matrix<T>& Matrix<T>::UPPERTriangular(){
  *this = upperTriangular();
  return *this;
}

template <typename T>
Matrix<T> Matrix<T>::lowerTriangular(){
  return *this;
}

template <typename T>
Matrix<T>& Matrix<T>::LOWERTriangular(){
  return *this;
}

template <typename T>
Matrix<T> Matrix<T>::ref(){
  return *this;
}

template <typename T>
Matrix<T>& Matrix<T>::REF(){
  return *this;
}

template <typename T>
Matrix<T> Matrix<T>::submatrix( uint row, uint col, uint rows, uint cols ){
  if( row + rows - 1 <= _m && col + cols - 1 <= _n && rows != 0 && cols != 0 ){
    Matrix result( rows, cols );
    for( uint i = 0; i < rows; i++ ){
      for( uint j = 0; j < cols; j++ ){
	result._data[i][j] = _data[i+row-1][j+col-1];
      }
    }
    return result;
  }
  else {
    _error = 1;
    return *this;
  }
}

template <typename T>
Matrix<T>& Matrix<T>::SUBMATRIX( uint row, uint col, uint rows, uint cols ){
  *this = submatrix( row, col, rows, cols );
  return *this;
}

template <typename T>
Matrix<T> Matrix<T>::add( const Matrix& rhs ){
  if( _m == rhs._m && _n == rhs._n ){
    Matrix result = *this;
    for( uint i = 0; i < _m; i++ ){
      for( uint j = 0; j < _n; j++ ){
	result._data[i][j] = result._data[i][j] + rhs._data[i][j];
      }
    }
    return result;
  }
  else {
    _error = 1;
    return *this;
  }
}

template <typename T>
Matrix<T> Matrix<T>::operator + ( const Matrix& rhs ){
  return add( rhs );
}


template <typename T>
Matrix<T>& Matrix<T>::ADD( const Matrix& rhs ){
  *this = *this + rhs;
  _det_up_to_date = 0;
  return *this;
}

template <typename T>
Matrix<T>& Matrix<T>::operator += ( const Matrix& rhs ){
  return ADD( rhs );
}

template <typename T>
Matrix<T> Matrix<T>::sub( const Matrix& rhs ){
  Matrix result = *this;
  return result.add( -rhs );
}

template <typename T>
Matrix<T> Matrix<T>::operator - ( const Matrix& rhs ){
  return sub( rhs );
}

template <typename T>
Matrix<T>& Matrix<T>::SUB( const Matrix& rhs ){
  return *this += -rhs;
}

template <typename T>
Matrix<T>& Matrix<T>::operator -= ( const Matrix& rhs ){
  return SUB( rhs );
}

template <typename T>
Matrix<T> Matrix<T>::mul( const Matrix& rhs ){
  if( _n == rhs._m ){
    Matrix result( _m, rhs._n );
    for( uint i = 0; i < _m; i++ ){
      for( uint j = 0; j < rhs._n; j++ ){
	int sum = 0;
	for( uint k = 0; k < _n; k++ ){
	  sum = sum + ( _data[i][k] * rhs._data[k][j] );
	}
	result._data[i][j] = sum;
      }
    }
    return result;
  }
  else {
    _error = 1;
    return *this;
  }
}

template <typename T>
Matrix<T> Matrix<T>::operator * ( const Matrix& rhs ){
  return mul( rhs );
}

template <typename T>
Matrix<T> Matrix<T>::mul( const T& in ){
  Matrix result = *this;
  for( uint i = 0; i < _m; i++ ){
    for( uint j = 0; j < _n; j++ ){
      result._data[i][j] = result._data[i][j] * in;
    }
  }
  return result;
}

template <typename T>
Matrix<T> Matrix<T>::operator * ( const T& in ){
  return mul( in );
}


template <typename T>
Matrix<T>& Matrix<T>::MUL( const Matrix& rhs ){
  *this = *this * rhs;
  _det_up_to_date = 0;
  return *this;
}

template <typename T>
Matrix<T>& Matrix<T>::operator *= ( const Matrix& rhs ){
  return MUL(rhs);
}

template <typename T>
Matrix<T>& Matrix<T>::MUL( const T& in ){
  *this = *this * in;
  _det_up_to_date = 0;
  return *this;
}

template <typename T>
Matrix<T>& Matrix<T>::operator *= ( const T& in ){
  return MUL( in );
}

template <typename T>
Matrix<T> Matrix<T>::div( const Matrix& rhs ){return *this;}

template <typename T>
Matrix<T> Matrix<T>::operator / ( const Matrix& rhs ){return *this;}

template <typename T>
Matrix<T> Matrix<T>::div( const T& in ){return *this;}

template <typename T>
Matrix<T> Matrix<T>::operator / ( const T& in ){return *this;}

template <typename T>
Matrix<T>& Matrix<T>::DIV( const Matrix& rhs ){return *this;}

template <typename T>
Matrix<T>& Matrix<T>::operator /= ( const Matrix& rhs ){return *this;}

template <typename T>
Matrix<T>& Matrix<T>::DIV( const T& in ){return *this;}

template <typename T>
Matrix<T>& Matrix<T>::operator /= ( const T& in ){return *this;}

//If power is 0, what does this return?
template <typename T>
Matrix<T> Matrix<T>::exp( const int power ){
  if( _m == _n && power > 0){
    Matrix result = *this;
    for( int i = 1; i < power; i++ ){
      result *= *this;
    }
    return result;
  }
  else {
    if( power == -1 ){ //Special case: Return the inverse.
      //return ~*this;
      return *this;
    }
    else {
      _error = 1;
      return *this;
    }
  }
}

template <typename T>
Matrix<T> Matrix<T>::operator ^ ( const int power ){
  return exp( power );
}

template <typename T>
Matrix<T>& Matrix<T>::EXP( const int power ){
  if( _m == _n && power > 0){
    Matrix result = *this;
    for( int i = 1; i < power; i++ ){
      *this *= result;
    }
    return *this;
  }
  else {
    if( power == -1 ){ //Special case: Return the inverse.
      //return !*this;
      return *this;
    }
    else {
      _error = 1;
      return *this;
    }
  }
}

template <typename T>
Matrix<T>& Matrix<T>::operator ^= ( const int power ){
  return EXP( power );
}

template <typename T>
Matrix<T> Matrix<T>::inv(){
  if( _m == _n ){
    //*****Create a matching identity matrix to mirror our row operations.
    Matrix identity( _m, _n );
    for( uint i = 0; i < _m; i++ )
      identity._data[i][i] = 1;

    Matrix result = *this;
    //*****For each leading 1...
    for( uint i = 0; i < _m; i++ ){
      //*****Put a leading nonzero in the current row.
      if( result._data[i][i] == 0 ){
	for( uint j = i+1; j < _m; j++ ){
	  if( result._data[j][i] != 0 ){
	    identity.SWITCHRows( i+1, j+1 );
	    result.SWITCHRows( i+1, j+1 );
	    break;
	  }
	}
      }
      if( result._data[i][i] == 0 ){ //All 0s in the column. No inverse possible.
	_error = 1;
	return *this;
      }
      //*****Multiply current row by 1/value to give it a leading 1.
      T value = result._data[i][i];
      result.MULTIPLYRow( 1/value, i+1 );
      identity.MULTIPLYRow( 1/value, i+1 );
      //*****Clean the column so the leading 1 is the only number in the column.
      Matrix res( result, i );
      Matrix ide( identity, i );
      for( uint j = 0; j < _m; j++ ){
	if( i != j ){
	  T coeff = result._data[j][i];
	  result.ADDToRow( (res*-coeff), j+1 );
	  identity.ADDToRow( (ide*-coeff), j+1 );
	}
      }
    }
    return identity;
  }
  else {
    _error = 1;
    return *this;
  }
}
template <typename T>
Matrix<T> Matrix<T>::operator ~ (){
  return inv();
}

template <typename T>
Matrix<T>& Matrix<T>::INV(){
  *this = ~*this;
  return *this;
}
template <typename T>
Matrix<T>& Matrix<T>::operator ! (){
  return INV();
}

template <typename T>
Matrix<T> Matrix<T>::trans(){
  Matrix result( _n, _m );
  for( uint i = 0; i < _m; i++ ){
    for( uint j = 0; j < _n; j++ ){
      result._data[j][i] = _data[i][j];
    }
  }
  return result;
}
template <typename T>
Matrix<T> Matrix<T>::operator + (){
  return trans();
}

template <typename T>
Matrix<T>& Matrix<T>::TRANS(){
  *this = +*this;
  return *this;
}

template <typename T>
Matrix<T>& Matrix<T>::operator ++ (){
  return TRANS();
}

template <typename T>
T Matrix<T>::det(){
  if( _det_up_to_date )
    return _determinant;
  else {
    if( _m != _n ){
      return 0;
    }
    _det_up_to_date = true;
    _determinant = _det( *this );
    return _determinant;
  }
}

template <typename T>
T Matrix<T>::_det( Matrix in ){
  uint rows = in._m;
  uint cols = in._n;
  if( rows == 2 ){
    return in._data[0][0]*in._data[1][1] - in._data[1][0]*in._data[0][1];
  }
  if( rows == 1 ){
    return in._data[0][0];
  }
  T sum = 0.0;
  int sign = 1;
  for( uint iter = 0; iter < in._m; iter++ ){
    Matrix out( rows - 1, cols - 1 );
    for( uint i = 1; i < in._m; i++ ){
      for( uint j = 0; j < in._n; j++ ){
	if( j != iter ){
	  out << in._data[i][j];
	}
      }
    }
    sum = sum + in._data[0][iter] * sign * _det( out );
    sign = sign * -1;
  }
  return sum;
}

template <typename T>
void Matrix<T>::backSubstitution(Matrix& b){
  for( uint i = rows()-1; i > -1; i-- ){
    for( uint j = 0; j < rows(); j++ ){
      if( j != i ){
	addToRow( Matrix( _data[i] ) * _data[j][i], j );
	b.addToRow( Matrix( b._data[i] ) * b._data[j][i], j );
      }
    }
  }
}

template <typename T>
void Matrix<T>::forwardSubstitution(Matrix& b){
  for( uint i = rows()-1; i > -1; i++ ){
    for( uint j = 0; j < rows(); j++ ){
      if( j != i ){
	addToRow( Matrix( _data[i] ) * _data[j][i] , j );
	b.addToRow( Matrix( b._data[i] ) * b._data[j][i] , j );
      }
    }
  }
}
