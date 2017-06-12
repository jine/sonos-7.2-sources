void 	rprintf_init(void);
void	rprintf_release(void);
int	rprintf( const char *fmt, ... );
void    my_rprintf( void *ah, const char *fmt, ... );
void    my_nrprintf( void *ah, int level, const char *fmt, ... );

