#include "matrix_compare_perf_test.hpp"

#include <vmmlib/matrix.hpp>
#include <vmmlib/matrix4.h>


namespace vmml
{


void
matrix_compare_perf_test::run()
{
    double Mdata4x4[] = { 17., 24., 1., 8., 23., 5., 7., 14.,
     4., 6., 13., 20., 10., 12., 19., 21. };
    
    matrix< 4, 4 > m, m_inverse;
    m = Mdata4x4;
    
    Matrix4< double > m_old, m_old_inverse;
    m_old.set( Mdata4x4 );
    
    
    new_test( "matrix 4x4 inversion comparison" );
    start( "old" );
    for( size_t i = 0; i < 1e6; ++i )
    {
        m_old.getInverse( m_old_inverse );
    }
    stop();

    start( "new" );
    for( size_t i = 0; i < 1e6; ++i )
    {
        m.computeInverse( m_inverse );
    }
    stop();
    compare();
    
    
    double d;

    new_test( "operator[]" );
    start( "old 1d[]" );
    for( size_t i = 0; i < 1e6; ++i )
    {
        d = m_old.array[ i % 16 ];
    }
    stop();

    start( "new 1d[]()" );
    for( size_t i = 0; i < 1e6; ++i )
    {
        d = m.array[ i % 16 ];
    }
    stop();
    compare();

    new_test( "operator[] 2d" );
    {
        start( "old 2d[]" );
        for( size_t i = 0; i < 1e6; ++i )
        {
            d = m_old.m[ i % 4 ][ i % 4 ];
        }
        stop();

        start( "new 2d[]()" );
        for( size_t i = 0; i < 1e6; ++i )
        {
            d = m[ i % 4 ][ i % 4 ];
        }
        stop();
    }
    compare();



    new_test( "operator[]" );
    start( "new []" );
    for( size_t i = 0; i < 1e6; ++i )
    {
        d = m[ i % 3 ][ i % 3 ];
    }
    stop();

    start( "new at()" );
    for( size_t i = 0; i < 1e6; ++i )
    {
        d = m.at( i % 3, i % 3 );
    }
    stop();
    compare();
    
    

    new_test( "copy ctor" );
    start( "old" );
    for( size_t i = 0; i < 1e6; ++i )
    {
        Matrix4< double > mm( m_old );
    }
    stop();

    start( "new" );
    for( size_t i = 0; i < 1e6; ++i )
    {
        matrix< 4, 4, double > mm( m );
    }
    stop();
    compare();



    new_test( "operator==/!=" );
    {
        Matrix4< double > mm_old( m_old );
        matrix< 4, 4, double > mm( m );
        bool equal;
        start( "old" );
            for( size_t i = 0; i < 1e6; ++i )
            {
                equal = mm_old != m_old;
            }
        stop();

        start( "new" );
            for( size_t i = 0; i < 1e6; ++i )
            {
                equal = mm != m;
            }
        stop();
    }
    compare();


    new_test( "setting to identity" );
    {
        Matrix4< double > mm_old;
        matrix< 4, 4, double > mm;

        start( "old" );
        for( size_t i = 0; i < 1e6; ++i )
        {
            mm_old = Matrix4< double >::IDENTITY;
        }
        stop();

        start( "new" );
        for( size_t i = 0; i < 1e6; ++i )
        {
            //mm.identity();
            mm = matrix< 4, 4, double >::IDENTITY;
        }
        stop();
        
    }
    compare();


    new_test( "setting to zero" );
    {
        Matrix4< double > mm_old;
        matrix< 4, 4, double > mm;

        start( "old" );
        for( size_t i = 0; i < 1e6; ++i )
        {
            mm_old = Matrix4< double >::ZERO;
        }
        stop();

        start( "new" );
        for( size_t i = 0; i < 1e6; ++i )
        {
            //mm.identity();
            mm = matrix< 4, 4, double >::ZERO;
        }
        stop();
        
    }
    compare();
    
    

}

} // namespace vmml

