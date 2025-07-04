using CSV
using DataFrames

# chemin du fichier 
path = "./Plot/Milestone 2/Résult 9/"
file = "walkingPattern_ref.csv"

# Charger le fichier CSV
df = CSV.read(path*file, DataFrame)

# Conserver les deux premières lignes
df_head = df[1:5:end, :]

# Échantillonner 1 ligne sur 5 à partir de la 3e ligne
# df_rest = df[6:5:end, :]

# Combiner les deux
# df_sampled = vcat(df_head, df_rest)

# Sauvegarder le résultat
CSV.write(path*"walkingPattern_ref_100ms.csv", df_head)
