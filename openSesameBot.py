import discord

class Client(discord.Client):
    async def on_read(self):
        print(f'Logged on as {self.user}!');
        
intents = discord.Intents.default();
intents.message_content = True;



#run bot
client = Client(intents = intents);

#will fill in after to avoid secret being in repo
client.run('');
